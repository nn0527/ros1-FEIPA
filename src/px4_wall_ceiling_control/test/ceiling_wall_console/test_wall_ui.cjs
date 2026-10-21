const assert=require('node:assert/strict');
const {wallView}=require('../../web/ceiling_wall_console/app.js');
function snapshot(){
  const values={runtime:{enabled:true},state:{armed:true,connected:true,mode:'ALTCTL'},odom:{},estimator:{},operator:{mode:1},phase:'IDLE',owner:0,kill:{valid:true,active:false},wall:{state_name:'IDLE'},setpoint:{thrust:.49}};
  return {status:Object.fromEntries(Object.entries(values).map(([key,value])=>[key,{fresh:true,value}])),ssh:{connected:true},session:{control:true,feedback_live:true,push:true},can_wall:true,can_detach:true,can_cancel_wait:false,blockers:[],detach_blockers:[]};
}
let checks=0;const check=condition=>{assert.ok(condition);checks++;};
check(wallView(snapshot()).canWall);check(!wallView(snapshot(),true).canWall);
for(const key of Object.keys(snapshot().status).filter(k=>k!=='setpoint')){const s=snapshot();s.status[key].fresh=false;check(!wallView(s).canWall);check(!wallView(s).canDetach);}
const stale=snapshot();stale.status.setpoint.fresh=false;check(!wallView(stale).canDetach);
for(const mutate of [s=>s.ssh.connected=false,s=>s.session.feedback_live=false,s=>s.session.control=false,s=>s.status.kill.value.active=true]){const s=snapshot();mutate(s);check(!wallView(s).canWall);check(!wallView(s).canDetach);}
const hold=snapshot();hold.status.wall.value={state_name:'WALL_HOLD',hold_achieved:true};hold.status.state.value.mode='OFFBOARD';hold.status.phase.value='ACTIVE';hold.status.owner.value=2;hold.status.operator.value.mode=3;hold.success=true;
check(wallView(hold).holding);hold.session.control=false;check(wallView(hold).holding);check(!wallView(hold).canDetach);
hold.session.feedback_live=false;check(!wallView(hold).holding);check(wallView(hold).holdAchieved);
const returned=snapshot();returned.status.wall.value={state_name:'IDLE',detach_completed:true};check(wallView(returned).returned);returned.status.state.fresh=false;check(!wallView(returned).returned);
const wait=snapshot();wait.can_cancel_wait=true;wait.status.phase.value='WAITING_WALL';wait.status.operator.value.mode=3;check(wallView(wait).canCancel);wait.session.feedback_live=false;check(!wallView(wait).canCancel);

// Removing the standalone action must preserve normal automatic handback feedback.
for(const mode of ['ALTCTL','POSCTL']){
  const returning=snapshot();returning.status.runtime.value.wall_pilot_mode=mode;
  returning.status.phase.value='RETURNING';returning.status.owner.value=2;
  returning.status.state.value.mode='OFFBOARD';
  check(wallView(returning).step===8);check(wallView(returning).reason.includes(mode));
  returning.status.wall.value.detach_completed=true;returning.status.phase.value='IDLE';
  returning.status.owner.value=0;returning.status.state.value.mode=mode;
  check(wallView(returning).returned);check(wallView(returning).reason.includes(mode));
}
const fs=require('node:fs'),path=require('node:path');
const html=fs.readFileSync(path.join(__dirname,'../../web/ceiling_wall_console/index.html'),'utf8');
const script=fs.readFileSync(path.join(__dirname,'../../web/ceiling_wall_console/app.js'),'utf8');
check(!html.includes('id="rcTakeover"'));check(!html.includes('一键遥控接管'));
check(!script.includes('/api/rc-takeover'));check(!script.includes('takeoverView'));
console.log(`${checks} wall UI and automatic handback assertions passed`);
