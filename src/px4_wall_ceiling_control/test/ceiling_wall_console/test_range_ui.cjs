const assert=require('node:assert/strict');
const {rangeView,wallView}=require('../../web/ceiling_wall_console/app.js');
function snapshot(){
  const values={runtime:{enabled:true,wall_pilot_mode:'POSCTL'},state:{connected:true,armed:true,mode:'OFFBOARD'},
    wall:{state_name:'RECOVER',detach_range_height_enabled:true,detach_task_phase_name:'RANGE_HEIGHT_CONTROL',
      detach_range_target_locked:true,detach_range_manual_available:true,detach_range_manual_target_m:1.5},
    kill:{valid:true,active:false},owner:2,setpoint:{thrust:.5},phase:'ACTIVE',operator:{mode:3},odom:{},estimator:{}};
  return {config:{detach_range_manual_target_m:1.5},status:Object.fromEntries(Object.entries(values).map(([k,v])=>[k,{value:v,fresh:true}])),
    ssh:{connected:true},session:{feedback_live:true,push:true,control:true},can_set_range_height:true};
}
let count=0;
function check(condition){assert.ok(condition);count++;}
check(rangeView(snapshot()).canSetHeight);
check(rangeView(snapshot()).active);
check(!rangeView(snapshot(),true).canSetHeight);
for(const change of [
  s=>s.ssh.connected=false,s=>s.session.feedback_live=false,s=>s.session.push=false,s=>s.session.control=false,
  s=>s.status.state.fresh=false,s=>s.status.wall.fresh=false,s=>s.status.kill.fresh=false,s=>s.status.owner.fresh=false,
  s=>s.status.setpoint.fresh=false,s=>s.status.phase.fresh=false,s=>s.status.phase.value='RC_TAKEOVER',
  s=>s.status.owner.value=1,s=>s.status.state.value.mode='POSCTL',s=>s.status.state.value.armed=false,
  s=>s.status.kill.value.active=true,s=>s.status.kill.value.valid=false,s=>s.can_set_range_height=false,
  s=>s.status.wall.value.detach_range_manual_available=false]){
  const s=snapshot();change(s);check(!rangeView(s).canSetHeight);
}
for(const change of [s=>s.ssh.connected=false,s=>s.session.feedback_live=false,s=>s.session.push=false,s=>s.status.wall.fresh=false]){
  const s=snapshot();change(s);check(!rangeView(s).active);
}
const unavailable=snapshot();unavailable.can_set_range_height=false;unavailable.set_range_height_blockers=['本轮已提交'];
check(rangeView(unavailable).reason==='本轮已提交');
const diagnostic=snapshot();diagnostic.status.odom.value={finite:false,vertical_speed:NaN};
check(rangeView(diagnostic).canSetHeight); // The node's range admission owns this decision.
const stages={ROTATING_TO_RANGE_ENTRY:'阶段 1',RANGE_HEIGHT_CONTROL:'阶段 2',HANDOVER_READY:'阶段 3',FAULT:'离墙异常'};
for(const [phase,title] of Object.entries(stages)){
  const s=snapshot();s.status.wall.value.detach_task_phase_name=phase;
  check(wallView(s).title.startsWith(title));
}
const recovered=snapshot();recovered.status.wall.value={...recovered.status.wall.value,detach_task_phase_name:'COMPLETED',
  detach_range_manual_recovery:true,fault_recovery_completed:true,detach_completed:false,recovery_completed:false};
recovered.status.state.value.mode='POSCTL';recovered.status.owner.value=0;recovered.status.phase.value='IDLE';
check(wallView(recovered).returned);check(wallView(recovered).title.includes('人工定高恢复完成'));
recovered.status.state.fresh=false;check(!wallView(recovered).returned);
check(wallView(recovered).title.includes('等待 POSCTL'));
console.log(`${count} range phase, height and handover assertions passed`);
