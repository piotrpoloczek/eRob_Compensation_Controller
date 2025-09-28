const dgram = require('dgram');
const http = require('http');
const fs = require('fs');
const path = require('path');
const { WebSocketServer } = require('ws');

const HTTP_PORT = 9002;
const UDP_PORT  = 9999;
const UDP_HOST  = '127.0.0.1';

// tiny static file server
const server = http.createServer((req,res)=>{
  const p = req.url === '/' ? '/index.html' : req.url;
  const file = path.join(__dirname, p);
  if (fs.existsSync(file)) {
    res.writeHead(200, {'Content-Type': p.endsWith('.html')?'text/html':'text/plain'});
    return fs.createReadStream(file).pipe(res);
  }
  res.writeHead(404); res.end('not found');
});

const wss = new WebSocketServer({ server, path: '/stream' });
function broadcast(obj) {
  const msg = JSON.stringify(obj);
  wss.clients.forEach(ws => { if (ws.readyState === ws.OPEN) ws.send(msg); });
}

// UDP socket
const udp = dgram.createSocket('udp4');
udp.on('listening', ()=> console.log(`UDP listening ${UDP_HOST}:${UDP_PORT}`));
udp.on('error', (e)=> console.error('UDP error', e));
udp.bind(UDP_PORT, UDP_HOST);

// Simple per-slave cache → emit array at 20 Hz
const latest = new Map();
const TICK_MS = 50; // 20 Hz

setInterval(()=> {
  if (latest.size === 0) return;
  const samples = [...latest.values()].sort((a,b)=> a.slave - b.slave);
  broadcast({ samples });
}, TICK_MS);

udp.on('message', (buf)=> {
  if (buf.length < 32) return;
  if (buf[0]!==0x54 || buf[1]!==0x53 || buf[2]!==0x31 || buf[3]!==1) return; // 'T','S','1',v1
  const seq   = buf.readUInt32LE(4);
  const t_ns  = Number(buf.readBigUInt64LE(8));
  const slave = buf.readUInt16LE(16);
  const torque_mN_m      = buf.readInt32LE(18);
  const ratio_tenths_pct = buf.readInt16LE(22);

  latest.set(slave, {
    seq, t_ns, slave,
    torque_Nm: torque_mN_m / 1000.0,
    ratio_percent: ratio_tenths_pct / 10.0
  });
});

server.listen(HTTP_PORT, ()=> {
  console.log(`HTTP http://localhost:${HTTP_PORT} | WS ws://localhost:${HTTP_PORT}/stream`);
});
