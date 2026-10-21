const assert=require('node:assert/strict');
const {view}=require('../../web/ceiling_wall_console/app.js');
function snapshot(){
  const values={runtime:{enabled:true},state:{armed:true,connected:true,mode:'ALTCTL'},odom:{},estimator:{},operator:{},phase:'IDLE',owner:0,kill:{valid:true,active:false},ceiling_status:{state_name:'NORMAL_FLIGHT'},setpoint:{thrust:.45}};
  return {status:Object.fromEntries(Object.entries(values).map(([key,value])=>[key,{fresh:true,value}])),ssh:{connected:true},session:{control:true,feedback_live:true},can_ceiling:true,can_ceiling_detach:false,ceiling_blockers:[]};
}
let checks=0;
const check=(condition)=>{assert.ok(condition);checks++;};
check(view(snapshot()).canAttach);
check(!view(snapshot(),true).canAttach);
for(const key of Object.keys(snapshot().status).filter(k=>k!=='setpoint')){
  const s=snapshot();s.status[key].fresh=false;s.can_ceiling_detach=true;
  check(!view(s).canAttach);check(!view(s).canDetach);
}
for(const mutate of [s=>s.ssh.connected=false,s=>s.session.feedback_live=false,s=>s.session.control=false]){
  const s=snapshot();mutate(s);s.can_ceiling_detach=true;
  check(!view(s).canAttach);check(!view(s).canDetach);check(!view(s).canStart);
}
const hold=snapshot();hold.status.ceiling_status.value={state_name:'SURFACE_HOLD'};hold.can_ceiling=false;hold.can_ceiling_detach=true;
check(view(hold).canDetach);check(!view(hold).canAttach);check(view(hold).stage==='顶吸保持');
hold.status.setpoint.fresh=false;check(!view(hold).canDetach);
const initial=snapshot();initial.status.runtime.value.enabled=false;initial.status.state.value.armed=false;
check(view(initial).canStart);initial.status.state.value.armed=true;check(!view(initial).canStart);
initial.status.state.value.armed=false;initial.starting=true;check(!view(initial).canStart);
check(!view(snapshot()).editable);
const bootstrap=snapshot();bootstrap.status.runtime={fresh:false,value:null};bootstrap.status.state={fresh:false,value:null};check(view(bootstrap).canStart);
const returning=snapshot();returning.status.phase.value='RETURNING';check(view(returning).stage==='等待遥控接管');
console.log(`${checks} UI state assertions passed`);
