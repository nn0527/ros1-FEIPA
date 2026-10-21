const assert=require('node:assert/strict');
const {consoleAddress}=require('../../web/ceiling_wall_console/connect.js');
assert.equal(consoleAddress('172.20.10.2',8790),'http://172.20.10.2:8790/login');
assert.equal(consoleAddress('10.187.58.66','8790'),'http://10.187.58.66:8790/login');
assert.equal(consoleAddress(' 192.168.1.4 ','8791'),'http://192.168.1.4:8791/login');
for(const ip of ['','256.1.1.1','172.20.10','172.20.10.2/path','172.20.10.2@evil.test','javascript:alert(1)','0172.20.10.2'])assert.throws(()=>consoleAddress(ip,8790));
for(const port of [0,65536,'8790/path','1.5','-3',''])assert.throws(()=>consoleAddress('172.20.10.2',port));
console.log('16 connection address assertions passed');
