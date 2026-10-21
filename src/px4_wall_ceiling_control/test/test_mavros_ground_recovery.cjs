'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const scriptPath = path.join(__dirname, '../web/ceiling_wall_console/app.js');
const {maintenanceView} = require(scriptPath);
function state(fresh, armed, control = true) {
  return {ssh: {connected: true}, session: {control, owner: 'pilot', owner_connected: true},
    status: {state: {fresh, value: {armed, connected: true}}, runtime: {fresh: true, value: {enabled: true}}}};
}
let s = state(true, false);
assert(maintenanceView(s).canRepair); assert(maintenanceView(s).canStop);
s = state(false, true);
assert(maintenanceView(s).canRepair); assert(maintenanceView(s).canStop);
s = state(true, true);
assert(!maintenanceView(s).canRepair); assert(!maintenanceView(s).canStop);
s = state(false, true, false);
assert(maintenanceView(s).canRepair); assert(!maintenanceView(s).canStop);
s = state(true, false, false);
assert(!maintenanceView(s).canRepair);
s = state(true, false);
assert(maintenanceView(s, true).canRepair); assert(!maintenanceView(s, true).canStop);
assert(!maintenanceView(s, true, true).canRepair);
s.ssh.connected = false;
assert(!maintenanceView(s).canRepair); assert(!maintenanceView(s).canStop);
// A synchronous paint failure after acquiring the busy flag must not latch it.
const elements = new Map();
const context = {document: {getElementById(id) {
  if (!elements.has(id)) elements.set(id, {addEventListener() {}, classList: {toggle() {}}});
  return elements.get(id);
}}};
vm.createContext(context);
let script = fs.readFileSync(scriptPath, 'utf8');
script = script.replace('setInterval(poll,300);poll();', `
  latest={};paint=()=>{throw Error('simulated rendering failure');};poll=async()=>{};
  globalThis.check=async()=>{await action('/api/stop');return busy;};
`);
vm.runInContext(script, context);
context.check().then(busy => {
  assert.equal(busy, false);
  console.log('PASS: 7 recovery scenarios and request flag cleared after rendering failure');
}).catch(error => {console.error(error); process.exitCode = 1;});
