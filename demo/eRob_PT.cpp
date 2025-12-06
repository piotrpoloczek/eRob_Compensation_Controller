/* 
 * This program is an EtherCAT master implementation that initializes and configures EtherCAT slaves,
 * manages their states, and handles real-time data exchange. It includes functions for setting up 
 * PDO mappings, synchronizing time with the distributed clock, and controlling servomotors in 
 * various operational modes. The program also features multi-threading for real-time processing 
 * and monitoring of the EtherCAT network.
 */
//#include <QCoreApplication>


#include <stdio.h>
#include <string.h>
#include "eRob.h"
#include "ethercat.h"
#include <iostream>
#include <inttypes.h>
#include <time.h>
#include <unistd.h>
#include <sys/mman.h>  //  add this header file for mlockall

#include <sys/time.h>
#include <pthread.h>
#include <math.h>

#include <chrono>
#include <ctime>

#include <iostream>
#include <cstdint>

#include <sched.h>
#include <stdlib.h>
#include <termios.h>
#include <fcntl.h>  //  include F_GETFL and F_SETFL definitions
#include "log.h"
#include "motor_control.h"
#include "torque_pkt.h"
#include "udp_pub.h"


// --- add this line ---
static UdpPub g_pub;   // global UDP publisher

// Define constants for stack size and timing
#define stack64k (64 * 1024) // Stack size for threads
#define NSEC_PER_SEC 1000000000   // Number of nanoseconds in one second
#define EC_TIMEOUTMON 5000        // Timeout for monitoring in microseconds


/* Top of file (once) */
static FILE *g_csv = NULL;
static int   g_csv_decim = 0;



// Function to synchronize time with the EtherCAT distributed clock
void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime);
// Function to add nanoseconds to a timespec structure
void add_timespec(struct timespec *ts, int64 addtime);
void set_thread_affinity(pthread_t thread, int cpu_core);
int erob_step_1(void);
int erob_step_2(void);
int erob_map_rxpdo(void);
int erob_map_txpod(void);
int erob_step_4(void);
int erob_step_5(void);
int erob_step_6(void);
int erob_step_7(void);
int erob_step_8(void);
int erob_step_9(void);
int erob_test(void);
int kbhit(void);
void key_control(void);


//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////

// ---------------- GYM MODE SETTINGS ----------------

enum GymMode {
    GYM_OFF = 0,
    GYM_ZERO = 1,
    GYM_RESIST = 2
};

static int   g_gym_mode = GYM_OFF;

static float g_gym_const_Nm   = 0.2f;
static float g_gym_visc_Nm_s  = 0.005f;
static float g_gym_max_Nm     = 2.0f;

// torque sensor tare
static int32_t g_torque_zero_mNm = 0;
static int     g_tare_done = 0;


// ---- Position-aware torque baseline ----
static const int   GYM_BINS = 181;          // -90..+90 deg inclusive (1 deg bins)
static int32_t     g_zero_tbl_mNm[GYM_BINS];
static uint8_t     g_zero_valid[GYM_BINS];

static inline int clampi(int v, int lo, int hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

static inline int angle_to_bin(float angle_deg)
{
    int idx = (int)lroundf(angle_deg + 90.0f);
    return clampi(idx, 0, GYM_BINS - 1);
}


//////////////////////////////////

// -------- Startup SDO read helpers --------
static bool read_sdo_i32(uint16 slave, uint16 index, uint8 sub, int32_t &out)
{
    int size = sizeof(out);
    out = 0;
    int wkc = ec_SDOread(slave, index, sub, FALSE, &size, &out, EC_TIMEOUTRXM);
    return (wkc > 0 && size == (int)sizeof(out));
}

static bool read_sdo_i16(uint16 slave, uint16 index, uint8 sub, int16_t &out)
{
    int size = sizeof(out);
    out = 0;
    int wkc = ec_SDOread(slave, index, sub, FALSE, &size, &out, EC_TIMEOUTRXM);
    return (wkc > 0 && size == (int)sizeof(out));
}

static void print_torque_sensor_sdo(uint16 slave)
{
    int32_t torque_raw = 0;
    int16_t ratio_raw  = 0;

    if (read_sdo_i32(slave, 0x3B69, 0x00, torque_raw)) {
        printf("[BOOT] Slave %u SDO 0x3B69:00 Torque sensor raw = %d\n",
               slave, torque_raw);
    } else {
        printf("[BOOT] Slave %u FAILED SDO 0x3B69:00\n", slave);
    }

    if (read_sdo_i16(slave, 0x3B6A, 0x00, ratio_raw)) {
        printf("[BOOT] Slave %u SDO 0x3B6A:00 Torque ratio raw = %d (0.1%% units)\n",
               slave, ratio_raw);
    }
}

static float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}


static void gym_tare_update(int32_t torque_mNm_raw)
{
    const int N = 500;  // ~500 ms @ 1000 Hz
    static int64_t acc = 0;
    static int cnt = 0;

    if (g_tare_done) return;

    acc += torque_mNm_raw;
    cnt++;

    if (cnt >= N) {
        g_torque_zero_mNm = (int32_t)(acc / cnt);
        g_tare_done = 1;
        acc = 0;
        cnt = 0;

        ECAT_LOG("[GYM] Torque tare set to %d mN·m (%.3f N·m)\n",
                 g_torque_zero_mNm, g_torque_zero_mNm / 1000.0);
    }
}

static inline double torque_comp_Nm(int32_t torque_mNm_raw)
{
    return (double)(torque_mNm_raw - g_torque_zero_mNm) / 1000.0;
}


static void gym_zero_update_when_still(float angle_deg,
                                       float vel_dps,
                                       int32_t torque_raw_mNm)
{
    // "still" conditions
    const float vel_dead = 0.8f;      // deg/s
    if (fabsf(vel_dps) > vel_dead) {
        return;
    }

    int b = angle_to_bin(angle_deg);

    // First time → take immediately
    if (!g_zero_valid[b]) {
        g_zero_tbl_mNm[b] = torque_raw_mNm;
        g_zero_valid[b] = 1;
        return;
    }

    // Slow IIR update (very gentle)
    // This lets baseline adapt to drift and post-move settling
    const float alpha = 0.01f; // 1% per update @ 1000 Hz → stable but slowly adapting
    float z = (float)g_zero_tbl_mNm[b];
    z = z + alpha * ((float)torque_raw_mNm - z);
    g_zero_tbl_mNm[b] = (int32_t)lroundf(z);
}


static inline int32_t gym_zero_get(float angle_deg)
{
    int b = angle_to_bin(angle_deg);
    if (g_zero_valid[b]) {
        return g_zero_tbl_mNm[b];
    }

    // If not learned yet, fall back to 0
    // (You can also fall back to nearest valid bin later)
    return 0;
}


//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////

// Global variables for EtherCAT communication
char IOmap[4096]; // I/O mapping for EtherCAT
int expectedWKC; // Expected Work Counter
boolean needlf; // Flag to indicate if a line feed is needed
volatile int wkc; // Work Counter (volatile to ensure it is updated correctly in multi-threaded context)
boolean inOP; // Flag to indicate if the system is in operational state
uint8 currentgroup = 0; // Current group for EtherCAT communication
int dorun = 0; // Flag to indicate if the thread should run
bool start_ecatthread_thread; // Flag to start the EtherCAT thread
int ctime_thread; // Cycle time for the EtherCAT thread

int64 toff, gl_delta; // Time offset and global delta for synchronization

// Function prototypes for EtherCAT thread functions
OSAL_THREAD_FUNC ecat_check(void *ptr); // Function to check the state of EtherCAT slaves
OSAL_THREAD_FUNC ecat_thread(void *ptr); // Real-time EtherCAT thread function

static uint64_t samp_count = 0;
static double t0 = 0.0;
static int rate_init = 0;


// Thread handles for the EtherCAT threads
OSAL_THREAD_HANDLE thread1; // Handle for the EtherCAT check thread
OSAL_THREAD_HANDLE thread2; // Handle for the real-time EtherCAT thread

// Conversion units from the servomotor
float Cnt_to_deg = 0.000686645; // Conversion factor from counts to degrees


rxpdo_t rxpdo;
txpdo_t txpdo;
int step = 0;
int rdl; // Variable to hold read data length
uint16_t data_R;

int16_t SLAVE_ID = 1;


uint8_t exit_app = 0x00; // exit program


// main function
int main(int argc, char **argv) 
{
    MOTOR_CTRL_init();
    needlf = FALSE;
    inOP = FALSE;
    start_ecatthread_thread = FALSE;
    dorun = 0;
    ctime_thread = 1000;  // set cycle time to us

    // initialize the udp port for sending the torque sensor data
    g_pub.init("127.0.0.1", 9999);


    // set highest real-time priority
    struct sched_param param;
    param.sched_priority = 99;
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("sched_setscheduler failed");
    }
    // lock memory
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        perror("mlockall failed");
    }
    // set CPU affinity to two cores
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(2, &cpuset);  // use CPU core 2
    CPU_SET(3, &cpuset);  // use CPU core 3

    if (sched_setaffinity(0, sizeof(cpu_set_t), &cpuset) == -1) 
    {
        perror("sched_setaffinity");
        return EXIT_FAILURE;
    }
    ECAT_LOG("Running on CPU cores 2 and 3\n");
    erob_test();
    ECAT_LOG("End program\n");
    return EXIT_SUCCESS;
}

// Function prototype for the EtherCAT test function
int erob_test(void) 
{
    erob_step_1();
    erob_step_2();
    erob_map_rxpdo();
    erob_map_txpod();
    erob_step_4();
    erob_step_5();
    erob_step_6();
    erob_step_8();
    erob_step_9();
    osal_usleep(1e6);
    ec_close();
    ECAT_LOG("\n Request init state for all slaves\n");
    ec_slave[0].state = EC_STATE_INIT;
    /* request INIT state for all slaves */
    ec_writestate(0);
    ECAT_LOG("EtherCAT master closed.\n");
    return 0;
}

/* 
main thread for module control, including sending control torque, receiving position, speed, and current feedback
here requires real-time performance, no long-running logic is allowed
1000 Hz
 */
OSAL_THREAD_FUNC_RT ecat_thread(void *ptr)
{
    struct timespec ts, tleft;
    int ht;
    int64 cycletime;
    rxpdo_t *h_rx = &rxpdo;
    txpdo_t *h_tx = &txpdo;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    ht = (ts.tv_nsec / 1000000) + 1;
    ts.tv_nsec = ht * 1000000;
    if (ts.tv_nsec >= NSEC_PER_SEC) {
        ts.tv_sec++;
        ts.tv_nsec -= NSEC_PER_SEC;
    }

    cycletime = *(int *)ptr * 1000;   // us -> ns
    const float dt = (float)cycletime * 1e-9f;
    ECAT_LOG("cycletime:%.3fs\n", dt);

    toff = 0;
    dorun = 0;
    rate_init = 0;
    samp_count = 0;

    ec_send_processdata();

    // Per-slave velocity smoothing (safe small fixed array)
    // If you expect more than 16 slaves, raise this.
    static float vel_dps_f[16] = {0};

    while (exit_app == 0x00)
    {
        add_timespec(&ts, cycletime + toff);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);

        if (!start_ecatthread_thread) {
            continue;
        }

        wkc = ec_receive_processdata(EC_TIMEOUTRET);

        if (wkc >= expectedWKC)
        {
            // -----------------------------
            // Time source (FIXED ORDER)
            // -----------------------------
            double t_sec = 0.0;
            if (ec_slave[0].hasdc) {
                t_sec = (double)ec_DCtime * 1e-9;
            } else {
                struct timespec now;
                clock_gettime(CLOCK_MONOTONIC, &now);
                t_sec = (double)now.tv_sec + (double)now.tv_nsec * 1e-9;
            }

            // -----------------------------
            // Rate monitor (1 Hz print)
            // -----------------------------
            if (!rate_init) {
                t0 = t_sec;
                rate_init = 1;
            }

            samp_count++;
            double elapsed = t_sec - t0;
            if (elapsed >= 1.0) {
                double hz = samp_count / elapsed;
                ECAT_LOG("[RATE] Effective loop rate: %.1f Hz\n", hz);
                samp_count = 0;
                t0 = t_sec;
            }

            // Step advance once per cycle
            if (step < 8000) {
                step++;
            }

            // Iterate slaves
            for (int slave = 1; slave <= ec_slavecount; slave++)
            {
                // Copy this slave's inputs
                memcpy(&txpdo, ec_slave[slave].inputs, sizeof(txpdo_t));

                // Pull base command
                *h_rx = MOTOR_CTRL_get_cmd();

                const bool is_pt = (h_rx->mode_of_operation == 0x04);

                if (is_pt && g_gym_mode != GYM_OFF) {
                    // -------------------------
                // GYM MODE (single block)
                // -------------------------
                if (g_gym_mode != GYM_OFF)
                {
                    // Update tare once torque values are valid
                    gym_tare_update(txpdo.torque_mN_m);

                    const double torque_Nm_comp = torque_comp_Nm(txpdo.torque_mN_m);
                    const double ratio_percent  = txpdo.torque_ratio_pm / 10.0;

                    // Slow console prints (~2 Hz)
                    if ((g_csv_decim % 500) == 0) {
                        ECAT_LOG("[ECAT] Slave %d: Torque(comp) = %.3f N·m (raw %d mN·m), Ratio = %.1f%%\n",
                                 slave, torque_Nm_comp, txpdo.torque_mN_m, ratio_percent);
                    }

                    // -------------
                    // GYM_ZERO
                    // -------------
                    if (g_gym_mode == GYM_ZERO) {
                        h_rx->target_torque = 0;
                    }

                    else if (g_gym_mode == GYM_RESIST) {

                        // 1) Read raw velocity (as you already do)
                        const int32_t vel_cnt = txpdo.actual_velocity;
                        const float vel_dps_raw = vel_cnt * Cnt_to_deg;

                        // 2) Small smoothing for velocity
                        static float vel_dps_f = 0.0f;
                        const float alpha_v = 0.10f;
                        vel_dps_f = vel_dps_f + alpha_v * (vel_dps_raw - vel_dps_f);

                        // 3) User torque (already compensated around tare in your logic)
                        // If torque_comp_Nm() already subtracts tare, great.
                        // If not, use: torque_user = (txpdo.torque_mN_m - g_tare_mNm) * 1e-3;
                        float torque_user = (float)torque_comp_Nm(txpdo.torque_mN_m);

                        // 4) Filter torque a bit (faster than velocity filter)
                        static float torque_f = 0.0f;
                        const float alpha_t = 0.20f;
                        torque_f = torque_f + alpha_t * (torque_user - torque_f);

                        // 5) Intent thresholds
                        const float T_intent_Nm = 0.6f;   // start feeling you
                        const float T_release_Nm = 0.3f;  // hysteresis to avoid chatter

                        static bool intent = false;
                        if (!intent && fabsf(torque_f) > T_intent_Nm) intent = true;
                        if ( intent && fabsf(torque_f) < T_release_Nm) intent = false;

                        // 6) Build resist torque
                        float T_gym = 0.0f;

                        if (intent) {
                            const float sgn = (torque_f > 0) ? 1.0f : -1.0f;

                            // "Coulomb-like" term triggered by torque intent
                            T_gym += -(g_gym_const_Nm * sgn);

                            // Viscous term still tied to actual motion
                            T_gym += -(g_gym_visc_Nm_s * vel_dps_f);
                        }

                        T_gym = clampf(T_gym, -g_gym_max_Nm, +g_gym_max_Nm);

                        const int16_t T_cmd_mNm = (int16_t)lroundf(T_gym * 1000.0f);
                        h_rx->target_torque = T_cmd_mNm;
                    }


                    // -------------
                    // GYM_RESIST
                    // -------------
                    
                }
                }


                

                // -------------------------
                // Startup state machine
                // (overrides gym torque)
                // -------------------------
                if (step <= 2000) {
                    h_rx->controlword = 0x0080;
                    h_rx->target_torque = 0;
                } else if (step <= 2600) {
                    h_rx->controlword = 0x0006;
                    h_rx->target_torque = 0;
                } else if (step <= 3000) {
                    h_rx->controlword = 0x0007;
                    h_rx->target_torque = 0;
                } else if (step <= 3500) {
                    h_rx->controlword = 0x000F;
                    h_rx->target_torque = 0;
                } else {
                    h_rx->controlword = 0x000F;
                }

                // Copy outputs to this slave
                memcpy(ec_slave[slave].outputs, &rxpdo, sizeof(rxpdo_t));
            }

            // Advance decimator once per cycle
            g_csv_decim++;

            // Feedback uses last copied txpdo (OK for single slave)
            MOTOR_CTRL_set_fbk_raw(*h_tx);
        }

        // DC sync
        if (ec_slave[0].hasdc) {
            ec_sync(ec_DCtime, cycletime, &toff);
        }

        ec_send_processdata();
    }

    MOTOR_CTRL_exit();
}



/* 
 * PI calculation to synchronize Linux time with the Distributed Clock (DC) time.
 * This function calculates the offset time needed to align the Linux time with the DC time.
 */
void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime) {
    static int64 integral = 0; // Integral term for PI controller
    int64 delta; // Variable to hold the difference between reference time and cycle time
    delta = (reftime) % cycletime; // Calculate the delta time
    if (delta > (cycletime / 2)) {
        delta = delta - cycletime; // Adjust delta if it's greater than half the cycle time
    }
    if (delta > 0) {
        integral++; // Increment integral if delta is positive
    }
    if (delta < 0) {
        integral--; // Decrement integral if delta is negative
    }
    *offsettime = -(delta / 100) - (integral / 20); // Calculate the offset time
    gl_delta = delta; // Update global delta variable
}

/* 
 * Add nanoseconds to a timespec structure.
 * This function updates the timespec structure by adding a specified amount of time.
 */
void add_timespec(struct timespec *ts, int64 addtime) {
    int64 sec, nsec; // Variables to hold seconds and nanoseconds

    nsec = addtime % NSEC_PER_SEC; // Calculate nanoseconds to add
    sec = (addtime - nsec) / NSEC_PER_SEC; // Calculate seconds to add
    ts->tv_sec += sec; // Update seconds in timespec
    ts->tv_nsec += nsec; // Update nanoseconds in timespec
    if (ts->tv_nsec >= NSEC_PER_SEC) { // If nanoseconds exceed 1 second
        nsec = ts->tv_nsec % NSEC_PER_SEC; // Adjust nanoseconds
        ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC; // Increment seconds
        ts->tv_nsec = nsec; // Set adjusted nanoseconds
    }
}


/* 
 * EtherCAT check thread function
 * This function monitors the state of the EtherCAT slaves and attempts to recover 
 * any slaves that are not in the operational state.
 */
OSAL_THREAD_FUNC ecat_check(void *ptr)
{
    int slave; // Variable to hold the current slave index
    (void)ptr; // Not used
    struct timespec task_delay = {0, 1000000}; // 单位 ns, 0 秒 2 毫秒
    float dt_s = task_delay.tv_sec + task_delay.tv_nsec * 1e-9;
    ECAT_LOG("ecat_check task dt = %.4f s\n", dt_s);

    // Infinite loop for monitoring
    while(exit_app == 0x00) 
    { 
        // 电机控制相关
        MOTOR_CTRL_step(dt_s);

        if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf) {
                needlf = FALSE; // Reset line feed flag
                ECAT_LOG("\n"); // Print a new line
            }
            ec_group[currentgroup].docheckstate = FALSE; // Reset check state
            ec_readstate(); // Read the state of all slaves
            for (slave = 1; slave <= ec_slavecount; slave++) { // Loop through each slave
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL)) {
                    ec_group[currentgroup].docheckstate = TRUE; // Set check state if slave is not operational
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
                        ECAT_LOG("ERROR: Slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK); // Acknowledge error state
                        ec_writestate(slave); // Write the state change to the slave
                    } else if (ec_slave[slave].state == EC_STATE_SAFE_OP) {
                        ECAT_LOG("WARNING: Slave %d is in SAFE_OP, changing to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL; // Change state to operational
                        ec_writestate(slave); // Write the state change to the slave
                    } else if (ec_slave[slave].state > EC_STATE_NONE) {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON)) { // Reconfigure the slave if needed
                            ec_slave[slave].islost = FALSE; // Mark slave as found
                            ECAT_LOG("MESSAGE: Slave %d reconfigured\n", slave);
                        }
                    } else if (!ec_slave[slave].islost) {
                        ec_statecheck(slave, EC_STATE_OPERATIONAL,  EC_TIMEOUTRET); // Check the state of the slave
                        if (ec_slave[slave].state == EC_STATE_NONE) {
                            ec_slave[slave].islost = TRUE; // Mark slave as lost
                            ECAT_LOG("ERROR: Slave %d lost\n", slave);
                        }
                    }
                }
                if (ec_slave[slave].islost) { // If the slave is marked as lost
                    if (ec_slave[slave].state == EC_STATE_NONE) {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON)) { // Attempt to recover the lost slave
                            ec_slave[slave].islost = FALSE; // Mark slave as found
                            ECAT_LOG("MESSAGE: Slave %d recovered\n", slave);
                        }
                    } else {
                        ec_slave[slave].islost = FALSE; // Mark slave as found
                        ECAT_LOG("MESSAGE: Slave %d found\n", slave);
                    }
                }
            }
            if (!ec_group[currentgroup].docheckstate) {
                ECAT_LOG("OK: All slaves resumed OPERATIONAL.\n"); // Confirm all slaves are operational
            }
        }
        // 线程精确延时
        nanosleep(&task_delay, NULL);
    }
}



//##################################################################################################
// Function: Set the CPU affinity for a thread
void set_thread_affinity(pthread_t thread, int cpu_core) {
    cpu_set_t cpuset; // CPU set to specify which CPUs the thread can run on
    CPU_ZERO(&cpuset); // Clear the CPU set
    CPU_SET(cpu_core, &cpuset); // Add the specified CPU core to the set

    // Set the thread's CPU affinity
    int result = pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset);
    if (result != 0) {
        ECAT_LOG("Unable to set CPU affinity for thread %d\n", cpu_core); // Error message if setting fails
    } else {
        ECAT_LOG("Thread successfully bound to CPU %d\n", cpu_core); // Confirmation message if successful
    }
}

int erob_step_1(void)
{
    ECAT_LOG("__________STEP 1___________________\n");

    const char *env  = getenv("EC_IFACE");
    const char *used = env ? env : "eth0";


    /* Initialize EtherCAT master on the chosen interface */
    if (ec_init(used) <= 0) {
        ECAT_LOG("Error: Could not initialize EtherCAT on %s!\n", used);
        ECAT_LOG("Tip: run as root; NIC UP; unmanaged; or set EC_IFACE.\n");
        ECAT_LOG("___________________________________________\n");
        return -1;
    }

    ECAT_LOG("EtherCAT master initialized successfully on: %s\n", used);
    ECAT_LOG("___________________________________________\n");

    // Search for EtherCAT slaves on the network
    // Search for EtherCAT slaves on the network
    if (ec_config_init(FALSE) <= 0) {
        ECAT_LOG("Error: Cannot find EtherCAT slaves!\n");
        ECAT_LOG("___________________________________________\n");
        ec_close();
        return -1;
    }

    ECAT_LOG("%d slaves found and configured.\n", ec_slavecount);

    // ---- BOOT SDO sanity read ----
    for (int s = 1; s <= ec_slavecount; s++) {
        print_torque_sensor_sdo(s);
    }
    // ------------------------------

    ECAT_LOG("___________________________________________\n");
    return 0;

}

// 2. Change to pre-operational state to configure the PDO registers
int erob_step_2(void)
{
    ECAT_LOG("__________STEP 2___________________\n");

    // Check if the slave is ready to map
    ec_readstate(); // Read the state of the slaves

    for(int i = 1; i <= ec_slavecount; i++) { // Loop through each slave
        if(ec_slave[i].state != EC_STATE_PRE_OP) { // If the slave is not in PRE-OP state
            // Print the current state and status code of the slave
            ECAT_LOG("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                   i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
            ECAT_LOG("\nRequest init state for slave %d\n", i); // Request to change the state to INIT
            ec_slave[i].state = EC_STATE_INIT; // Set the slave state to INIT
            ECAT_LOG("___________________________________________\n");
        } else { // If the slave is in PRE-OP state
            ec_slave[0].state = EC_STATE_PRE_OP; // Set the first slave to PRE-OP state
            /* Request EC_STATE_PRE_OP state for all slaves */
            ec_writestate(0); // Write the state change to the slave
            /* Wait for all slaves to reach the PRE-OP state */
            if ((ec_statecheck(0, EC_STATE_PRE_OP,  3 * EC_TIMEOUTSTATE)) == EC_STATE_PRE_OP) {
                ECAT_LOG("State changed to EC_STATE_PRE_OP: %d \n", EC_STATE_PRE_OP);
                ECAT_LOG("___________________________________________\n");
            } else {
                ECAT_LOG("State EC_STATE_PRE_OP cannot be changed in step 2\n");
                return -1; // Return error if state change fails
            }
        }
    }
    return 0;
}

/**
 * 3.- Map RXPDO configure slave receive process data object RXPDO mapping (the data sent from the master to the slave) (from the joint module perspective, it is receiving)
 * SDO data service object
 * 0x1600~0x1603 is the number of the 1~4th receive object (Receive PDO 1 mapping) in the object dictionary
 * one PDO supports up to 255 data, use index to distinguish
 * index 0 needs to pass in the number of data 0-255,
 * index 1-255 needs to pass in 32-bit values, where
 * bit[0:7] is the length of the data, 0x08=8 bits; 0x10=16 bits, 0x20=32 bits
 * bit[8:15] is the sub-index of the data, if there is no sub-index, write 0
 * bit[15:31] is the address of the data
 * **/
int erob_map_rxpdo(void)
{
   
    ECAT_LOG("__________STEP 3: map rx pdo__________________\n");

    // Clear RXPDO mapping
    int retval = 0; // Variable to hold the return value of SDO write operations
    uint16 map_1c12; // Variable to hold the mapping for PDO
    uint8 zero_map = 0; // Variable to clear the PDO mapping
    uint32 map_object; // Variable to hold the mapping object
    uint16 clear_val = 0x0000; // Value to clear the mapping

    for(int i = 1; i <= ec_slavecount; i++) { // Loop through each slave
        // 1. First, disable PDO
        retval += ec_SDOwrite(i, 0x1600, 0x00, FALSE, sizeof(zero_map), &zero_map, EC_TIMEOUTSAFE);
        
        // 2. Configure new PDO mapping
        // Add Control Word control word
        map_object = 0x60400010;  // 0x6040:0 Control Word, 16 bits
        retval += ec_SDOwrite(i, 0x1600, 0x01, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
        
        // Add Target Torque target torque
        map_object = 0x60710010;  // 0x6071:0 Target Torque, 16 bits 
        // the value of the mapped object includes two parts, 0x6071 is the address of the target torque, 0x00 is the sub-index, 0x10 represents the torque is 16-bit data type
        // through the address of map_object, the value of the mapped object is passed in, during the execution of the erob_map_rxpdo function, this address is stable, when the ec_SDOwrite function is called, the memory of map_object is not released, so the ec_SDOwrite function can get the correct value according to this address.
        // only when the erob_map_rxpdo function is finished, the map_object variable will be released.
        // in the ec_SDOwrite function, the value of map_object is obtained through memory copy, memcpy(&SDOp->ldata[0], hp, psize);
        retval += ec_SDOwrite(i, 0x1600, 0x02, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
        
        // Add Torque Slope torque slope, set to 0 to disable slope setting, immediate response
        map_object = 0x60870020;  // 0x6087:0 Torque Slope, 32 bits
        retval += ec_SDOwrite(i, 0x1600, 0x03, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
        
        // Add Max Torque maximum torque
        map_object = 0x60720010;  // 0x6072:0 Max Torque, 16 bits
        retval += ec_SDOwrite(i, 0x1600, 0x04, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
        
        // Add Modes of Operation control mode
        map_object = 0x60600008;  // 0x6060:0 Modes of Operation, 8 bits
        retval += ec_SDOwrite(i, 0x1600, 0x05, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);

        map_object = 0x60B20010;  // 0x60B2:0 电流前馈值，int16_t current feedforward value
        retval += ec_SDOwrite(i, 0x1600, 0x06, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);

        map_object = 0x60FF0020;  // 0x60FF:0 速度目标值pul/s，int32_t speed target value
        retval += ec_SDOwrite(i, 0x1600, 0x07, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);

        map_object = 0x607F0020;  // 0x607F:0 最大允许速度pul/s，uint32_t maximum allowed speed
        retval += ec_SDOwrite(i, 0x1600, 0x08, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);

        map_object = 0x60800020;  // 0x6080:0 任何方向上允许的最大速度pul/s，uint32_t maximum allowed speed in any direction
        retval += ec_SDOwrite(i, 0x1600, 0x09, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);

        map_object = 0x60830020;  // 0x6083 轮廓加速度 pul/s，uint32_t contour acceleration
        retval += ec_SDOwrite(i, 0x1600, 0x0a, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);

        map_object = 0x60840020;  // 0x6084 轮廓减速度 pul/s，uint32_t
        retval += ec_SDOwrite(i, 0x1600, 0x0b, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
        
        // Add 8-bit padding 填充的位  
        /*
            因 ESC 芯片特性，如果 0x1600（RxPDO0）映射了 0x6060 或 0x1A00（TxPDO0）
            映射了 0x6061，则还需添加映射一个 8bit 长度的对象（此对象必须位于最后一 个
            子索引），使 PDO 总长度为偶数字节（16bit）对齐，否则 PDO 配置将无法生效，
            ESC 会报错 SM 配置无效。（参考手册第30页）
        */
        map_object = 0x00000008;  // 8-bit padding
        retval += ec_SDOwrite(i, 0x1600, 0x00c, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
        
        // Change the number of PDO mappings to 6 (including padding)
        uint8 map_count = 12;  // 映射10个数据
        retval += ec_SDOwrite(i, 0x1600, 0x00, FALSE, sizeof(map_count), &map_count, EC_TIMEOUTSAFE);
        
        // 4. Configure RXPDO allocation
        clear_val = 0x0000; // Clear the mapping
        retval += ec_SDOwrite(i, 0x1c12, 0x00, FALSE, sizeof(clear_val), &clear_val, EC_TIMEOUTSAFE);
        map_1c12 = 0x1600; // Set the mapping to the new PDO
        retval += ec_SDOwrite(i, 0x1c12, 0x01, FALSE, sizeof(map_1c12), &map_1c12, EC_TIMEOUTSAFE);
        map_1c12 = 0x0001; // Set the mapping index
        retval += ec_SDOwrite(i, 0x1c12, 0x00, FALSE, sizeof(map_1c12), &map_1c12, EC_TIMEOUTSAFE);
    }

    ECAT_LOG("PDO mapping configuration result: %d\n", retval);
    if (retval < 0) {
        ECAT_LOG("PDO mapping failed\n");
        return -1;
    }

    ECAT_LOG("RXPOD Mapping set correctly.\n");
    ECAT_LOG("___________________________________________\n");
    return 0;
}

/**
 * Map TXPOD 配置从站发送过程数据对象TXPDO映射（主站接收的从站的数据）
 * 这里定义了一个包含从站到主站数据的对象列表。所有 TxPDO 都位于索引
 * 0x1A00 到 0x1BFF 的对象字典中。
 * 索引0 字典中数据的个数0-254，
 * 索引1-255 字典中变量的地址，需要传入32位的值，其中
 * bit[0:7]是数据的长度, 0x08=8位数据; 0x10=16位数据，0x20=32位数据
 * bit[8:15]是数据的子索引，没有就写0
 * bit[15:31]是数据的地址
 * 
 * **/
int erob_map_txpod(void)
{
    int retval = 0; // Variable to hold the return value of SDO write operations
    uint16 map_1c13;
    uint16 clear_val = 0x0000; // Value to clear the mapping
    uint32 map_object; // Variable to hold the mapping objects
    ECAT_LOG("__________STEP 3: map tx pdo__________________\n");
    for(int i = 1; i <= ec_slavecount; i++) {
        // First, clear the TXPDO mapping
        clear_val = 0x0000;
        retval += ec_SDOwrite(i, 0x1A00, 0x00, FALSE, sizeof(clear_val), &clear_val, EC_TIMEOUTSAFE);

        // Configure TXPDO mapping entries
        // Status Word (0x6041:0, 16 bits)
        map_object = 0x60410010;
        retval += ec_SDOwrite(i, 0x1A00, 0x01, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);

        // Actual Position (0x6064:0, 32 bits)
        map_object = 0x60640020;
        retval += ec_SDOwrite(i, 0x1A00, 0x02, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);

        // Actual Velocity (0x606C:0, 32 bits)
        map_object = 0x606C0020;
        retval += ec_SDOwrite(i, 0x1A00, 0x03, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);

        // Actual Torque (0x6077:0, 16 bits)
        // 即电机实际电流(mA)=0x6078值×(0x6075值)/1000
        map_object = 0x60770010;
        retval += ec_SDOwrite(i, 0x1A00, 0x04, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);

        // After Actual Torque (0x6077:0, 16 bits)
        map_object = 0x3B690020;  // 0x3B69:0 Torque sensor value, DINT, 32 bits (mN·m)
        retval += ec_SDOwrite(i, 0x1A00, 0x05, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);

        map_object = 0x3B6A0010;  // 0x3B6A:0 Torque sensor ratio, INT, 16 bits (0.1 %)
        retval += ec_SDOwrite(i, 0x1A00, 0x06, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);


        // // 电流基值 额定电流mA (0x6075:0, uint16_t) 对应上位机安全电源界面：持续电流。电机额定电流，它取自电机基本参数。
        // 读取不到，有空再分析，先直接设置
        // map_object = 0x60750010;
        // retval += ec_SDOwrite(i, 0x1A00, 0x05, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);

        // Set the number of mapped objects (5 objects)
        // Update mapped count (we now have 6 entries)
        uint8 map_count = 6;       // or 7 if you also mapped 0x6076
        retval += ec_SDOwrite(i, 0x1A00, 0x00, FALSE, sizeof(map_count), &map_count, EC_TIMEOUTSAFE);

        // uint8 map_count = 4;
        // retval += ec_SDOwrite(i, 0x1A00, 0x00, FALSE, sizeof(map_count), &map_count, EC_TIMEOUTSAFE);

        // Configure TXPDO assignment
        // First, clear the assignment
        clear_val = 0x0000;
        retval += ec_SDOwrite(i, 0x1C13, 0x00, FALSE, sizeof(clear_val), &clear_val, EC_TIMEOUTSAFE);

        // Assign TXPDO to 0x1A00
        map_1c13 = 0x1A00;
        retval += ec_SDOwrite(i, 0x1C13, 0x01, FALSE, sizeof(map_1c13), &map_1c13, EC_TIMEOUTSAFE);

        // Set the number of assigned PDOs (1 PDO)
        map_1c13 = 0x0001;
        retval += ec_SDOwrite(i, 0x1C13, 0x00, FALSE, sizeof(map_1c13), &map_1c13, EC_TIMEOUTSAFE);
    }

    ECAT_LOG("Slave %d TXPDO mapping configuration result: %d\n", SLAVE_ID, retval);

    if (retval < 0) {
        ECAT_LOG("TXPDO Mapping failed\n");
        ECAT_LOG("___________________________________________\n");
        return -1;
    }

    ECAT_LOG("TXPDO Mapping set successfully\n");
    ECAT_LOG("___________________________________________\n");
    return 0;
}

 //4.- Set ecx_context.manualstatechange = 1. Map PDOs for all slaves by calling ec_config_map().
int erob_step_4(void)
{
    ECAT_LOG("__________STEP 4___________________\n");

    ecx_context.manualstatechange = 1; //Disable automatic state change
    osal_usleep(1e6); //Sleep for 1 second

    // Print the information of the slaves found
    for (int i = 1; i <= ec_slavecount; i++) {
        // (void)ecx_FPWR(ecx_context.port, i, ECT_REG_DCSYNCACT, sizeof(WA), &WA, 5 * EC_TIMEOUTRET);
        ECAT_LOG("Name: %s\n", ec_slave[i].name); //Print the name of the slave
        ECAT_LOG("Slave %d: Type %d, Address 0x%02x, State Machine actual %d, required %d\n", 
                i, ec_slave[i].eep_id, ec_slave[i].configadr, ec_slave[i].state, EC_STATE_INIT);
        ECAT_LOG("___________________________________________\n");
        ecx_dcsync0(&ecx_context, i, TRUE, 1000000, 0);  //Synchronize the distributed clock for the slave
    }

    // Map the configured PDOs to the IOmap
    ec_config_map(&IOmap);
    return 0;
}


 // Ensure all slaves are in PRE-OP state
int erob_step_5(void)
{
    ECAT_LOG("__________STEP 5___________________\n");
    // Ensure all slaves are in PRE-OP state
    for(int i = 1; i <= ec_slavecount; i++) {
        if(ec_slave[i].state != EC_STATE_PRE_OP) { // Check if the slave is not in PRE-OP state
            ECAT_LOG("Slave %d not in PRE-OP state. Current state: %d\n", i, ec_slave[i].state);
            return -1; // Return error if any slave is not in PRE-OP state
        }
    }

    // Configure Distributed Clock (DC)
    ec_configdc(); // Set up the distributed clock for synchronization

    // Request to switch to SAFE-OP state
    ec_slave[0].state = EC_STATE_SAFE_OP; // Set the first slave to SAFE-OP state
    ec_writestate(0); // Write the state change to the slave

    // Wait for the state transition
    if (ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4) == EC_STATE_SAFE_OP) {
        ECAT_LOG("Successfully changed to SAFE_OP state\n"); // Confirm successful state change
    } else {
        ECAT_LOG("Failed to change to SAFE_OP state\n");
        return -1; // Return error if state change fails
    }

    // Calculate the expected Work Counter (WKC)
    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC; // Calculate expected WKC based on outputs and inputs
    ECAT_LOG("Calculated workcounter %d\n", expectedWKC);

    // Read and display basic status information of the slaves
    ec_readstate(); // Read the state of all slaves
    for(int i = 1; i <= ec_slavecount; i++) {
        ECAT_LOG("Slave %d\n", i);
        ECAT_LOG("  State: %02x\n", ec_slave[i].state); // Print the state of the slave
        ECAT_LOG("  ALStatusCode: %04x\n", ec_slave[i].ALstatuscode); // Print the AL status code
        ECAT_LOG("  Delay: %d\n", ec_slave[i].pdelay); // Print the delay of the slave
        ECAT_LOG("  Has DC: %d\n", ec_slave[i].hasdc); // Check if the slave supports Distributed Clock
        ECAT_LOG("  DC Active: %d\n", ec_slave[i].DCactive); // Check if DC is active for the slave
        ECAT_LOG("  DC supported: %d\n", ec_slave[i].hasdc); // Print if DC is supported
    }

    // Read DC synchronization configuration using the correct parameters
    for(int i = 1; i <= ec_slavecount; i++) {
        uint16_t dcControl = 0; // Variable to hold DC control configuration
        int32_t cycleTime = 0; // Variable to hold cycle time
        int32_t shiftTime = 0; // Variable to hold shift time
        int size; // Variable to hold size for reading

        // Read DC synchronization configuration, adding the correct size parameter
        size = sizeof(dcControl);
        if (ec_SDOread(i, 0x1C32, 0x01, FALSE, &size, &dcControl, EC_TIMEOUTSAFE) > 0) {
            ECAT_LOG("Slave %d DC Configuration:\n", i);
            ECAT_LOG("  DC Control: 0x%04x\n", dcControl); // Print the DC control configuration
            
            size = sizeof(cycleTime);
            if (ec_SDOread(i, 0x1C32, 0x02, FALSE, &size, &cycleTime, EC_TIMEOUTSAFE) > 0) {
                ECAT_LOG("  Cycle Time: %d ns\n", cycleTime); // Print the cycle time
            }

        }
    }
    return 0;
}


int erob_step_6(void)
{
    uint8 WA = 0; //Variable for write access
    uint8 my_RA = 0; //Variable for read access
    uint32 TIME_RA; //Variable for time read access
    ECAT_LOG("__________STEP 6___________________\n");

    // Start the EtherCAT thread for real-time processing
    start_ecatthread_thread = TRUE; // Flag to indicate that the EtherCAT thread should start
    osal_thread_create_rt(&thread1, stack64k * 2, (void *)&ecat_thread, (void *)&ctime_thread); // Create the real-time EtherCAT thread
    // set_thread_affinity(*thread1, 4); // Optional: Set CPU affinity for the thread
    osal_thread_create(&thread2, stack64k * 2, (void *)&ecat_check, NULL); // Create the EtherCAT check thread
    // set_thread_affinity(*thread2, 5); // Optional: Set CPU affinity for the thread
    ECAT_LOG("___________________________________________\n");

    my_RA = 0; // Reset read access variable
    for (int cnt = 1; cnt <= ec_slavecount; cnt++) { // Loop through each slave
        ec_SDOread(cnt, 0x1c32, 0x01, FALSE, &rdl, &data_R, EC_TIMEOUTSAFE); // Read DC synchronization data
        ECAT_LOG("Slave %d DC synchronized 0x1c32: %d\n", cnt, data_R); // Print the synchronization data for each slave
    }
    return 0;
}

// 8. Transition to OP state
int erob_step_8(void)
{
    // 8. Transition to OP state
    ECAT_LOG("__________STEP 8___________________\n");

    // Send process data to the slaves
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET); // Receive process data and store the Work Counter

    // Set the first slave to operational state
    ec_slave[0].state = EC_STATE_OPERATIONAL; // Change the state of the first slave to OP
    ec_writestate(0); // Write the state change to the slave

    // Wait for the state transition to complete
    if ((ec_statecheck(0, EC_STATE_OPERATIONAL, 5 * EC_TIMEOUTSTATE)) == EC_STATE_OPERATIONAL) {
        ECAT_LOG("State changed to EC_STATE_OPERATIONAL: %d\n", EC_STATE_OPERATIONAL); // Confirm successful state change
        ECAT_LOG("___________________________________________\n");
    } else {
        ECAT_LOG("State could not be changed to EC_STATE_OPERATIONAL\n"); // Error message if state change fails
        for (int cnt = 1; cnt <= ec_slavecount; cnt++) {
            ECAT_LOG("ALstatuscode: %d\n", ecx_context.slavelist[cnt].ALstatuscode); // Print AL status codes for each slave
        }
    }

    // Read and display the state of all slaves
    ec_readstate(); // Read the state of all slaves
    for (int i = 1; i <= ec_slavecount; i++) {
        ECAT_LOG("Slave %d: Type %d, Address 0x%02x, State Machine actual %d, required %d\n", 
               i, ec_slave[i].eep_id, ec_slave[i].configadr, ec_slave[i].state, EC_STATE_OPERATIONAL); // Print slave information
        ECAT_LOG("Name: %s\n", ec_slave[i].name); // Print the name of the slave
        ECAT_LOG("___________________________________________\n");
    }
    return 0;
}

 // 9. Configure servomotor and mode operation
int erob_step_9(void)
{
    uint16_t Control_Word = 0x0000;
    uint8 operation_mode = 4;  // Profile Torque Mode
    uint16_t max_torque = 100;
    // 9. Configure servomotor and mode operation
    ECAT_LOG("__________STEP 9___________________\n");
    if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
        ECAT_LOG("######################################################################################\n");
        ECAT_LOG("################# All slaves entered into OP state! ##################################\n");

        for (int i = 1; i <= ec_slavecount; i++)
        {
            ec_SDOwrite(i, 0x6060, 0x00, FALSE, sizeof(operation_mode), &operation_mode, EC_TIMEOUTSAFE);
            ec_SDOwrite(i, 0x6072, 0x00, FALSE, sizeof(max_torque), &max_torque, EC_TIMEOUTSAFE);
            ec_SDOwrite(i, 0x6040, 0x00, FALSE, sizeof(Control_Word), &Control_Word, EC_TIMEOUTSAFE);
        }
        // 主循环
        for(uint32_t i = 1; i <= 3 * 60 * 60 * 1000; i++) 
        {
            osal_usleep(1000);
        }
    }
    return 0;
}

