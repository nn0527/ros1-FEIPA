"use strict";
const fresh=(s,key)=>s.status?.[key]?.fresh?s.status[key].value:null;
function view(s,busy=false){
  const live=!!s.ssh?.connected && !!s.session?.feedback_live;
  const control=live && !!s.session?.control;
  const runtime=fresh(s,"runtime"), state=fresh(s,"state"), ceiling=fresh(s,"ceiling_status"), phase=fresh(s,"phase"), owner=fresh(s,"owner"), kill=fresh(s,"kill");
  const enabled=!!runtime?.enabled;
  const reasons=(s.ceiling_blockers||[]).map(reason=>reason.replace("贴墙请求需", "顶吸请求需"));
  if(!live)reasons.unshift("实时反馈未连接，动作禁用");
  else if(!control)reasons.unshift("请先申请控制权");
  const required=["runtime","state","odom","estimator","operator","phase","owner","kill","ceiling_status"];
  const complete=required.every(key=>fresh(s,key)!==null);
  const editable=control && !enabled && !s.starting && !busy && !state?.armed;
  let stage="等待基础链路",reason=s.launch_error||"应用初始条件后，由飞手起飞并稳定",step=-1;
  const names={NORMAL_FLIGHT:"顶吸待命",CEILING_ARMED:"准备顶吸",APPROACH:"向上接近",ATTACH_CONTROL:"确认顶部接触",SURFACE_HOLD:"顶吸保持",DETACH:"脱离顶部",RECOVERY_HOVER:"恢复悬停",FAULT:"顶吸故障"};
  const steps={NORMAL_FLIGHT:0,CEILING_ARMED:1,APPROACH:2,ATTACH_CONTROL:2,SURFACE_HOLD:3,DETACH:4,RECOVERY_HOVER:4};
  if(s.starting){stage="基础链路启动中";reason="等待 MAVROS、雷达和控制节点就绪";}
  else if(enabled && ceiling){stage=names[ceiling.state_name]||ceiling.state_name;reason=ceiling.reason||"等待节点反馈";step=steps[ceiling.state_name]??-1;}
  if(["PRESTREAM","ENTERING"].includes(phase)){stage="正在接管飞行";reason=fresh(s,"reason")||"等待飞控确认 OFFBOARD";step=1;}
  if(phase==="RETURNING"){stage="等待遥控接管";reason="等待飞控确认 ALTCTL，当前尚未完成交接";step=5;}
  if(phase==="LOCKED"){stage="动作受阻";reason=fresh(s,"reason")||"请检查飞行状态";step=-1;}
  if(ceiling?.fault_detected){stage="顶吸故障";reason=ceiling.reason;step=-1;}
  if(kill?.active){stage="急停已生效";reason="动作禁用，请核对飞控状态";step=-1;}
  if(!live){stage="等待实时反馈";reason=s.ssh?.error||"反馈中断，无法确认当前飞行状态";step=-1;}
  else if(enabled && !ceiling){stage="顶吸反馈过期";reason="等待新的控制状态，动作禁用";step=-1;}
  let detachReason="仅在顶吸保持、OFFBOARD 和有效输出下接受脱离请求";
  if(!live)detachReason="实时反馈未连接";
  else if(!control)detachReason="请先申请控制权";
  else if(!complete)detachReason="脱离所需反馈缺失或过期";
  else if(ceiling?.height_source==="barometer_hold_relative" && (!ceiling.baro_fresh||!ceiling.baro_reference_ready))detachReason="气压数据或保持高度参考未就绪";
  else if(s.can_ceiling_detach)detachReason="脱离入口已就绪";
  const canAttach=control && complete && !busy && !!s.can_ceiling;
  const canDetach=control && complete && !!fresh(s,"setpoint") && !busy && !!s.can_ceiling_detach;
  const canRepair=maintenanceView(s,busy).canRepair;
  return {live,control,runtime,state,ceiling,phase,owner,kill,enabled,editable,canStart:editable,canRepair,canAttach,canDetach,reasons,detachReason,stage,reason,step};
}
function maintenanceView(s,busy=false,recoveryBusy=false){
  const linked=!!s.ssh?.connected,flight=fresh(s,"state"),armed=!!flight?.armed;
  const recovering=recoveryBusy||s.mavros_repair?.state==="RUNNING";
  const recoveryAuthority=!!s.session?.control||!flight?.connected||!s.session?.owner||s.session?.owner_connected===false;
  const repairReason=!linked?"测试台反馈服务未连接，等待连接恢复":recovering?"正在恢复，请等待结果":armed?"飞控仍已解锁，请先落地上锁":s.starting?"正在启动，请先停止基础链路":!recoveryAuthority?"其他在线设备持有控制权，请由控制端操作":"";
  const stopReason=!linked?"测试台反馈服务未连接，等待连接恢复":recovering?"正在恢复，请等待结果":!s.session?.control?"停止需要控制权；失联时可使用地面恢复":armed?"飞控仍已解锁，请先落地上锁":busy?"其他操作等待回执":"";
  return {canRepair:!repairReason,canStop:!stopReason,repairReason,stopReason};
}
const wallTranslations = {
  initialized:"等待贴墙指令", "PX4 entered OFFBOARD":"飞控已进入 OFFBOARD",
  "flight state timeout":"飞控心跳已过期", "MAVROS disconnected":"MAVROS 连接中断",
  "vehicle disarmed":"飞控未解锁", "local odometry timeout":"里程计数据已过期",
  "estimator status timeout":"估计器状态已过期", "estimator status unavailable":"未收到估计器状态",
  "front sensor invalid":"前向雷达无效", "ceiling sensor invalid":"顶部雷达无效",
  "sensor health timeout":"雷达健康状态已过期", "OFFBOARD setpoint stream lost":"姿态设定值流中断",
  "operator request released; retry enabled":"动作请求已释放，可以重新请求",
  "PX4 OFFBOARD entry timed out":"进入 OFFBOARD 超时", "attitude or vertical velocity estimate invalid":"姿态或竖直速度估计无效",
  "front detect safety failure":"检测墙面时输入或飞行状态无效", "hover stabilized":"悬停已稳定",
  "approach safety failure":"接近墙面时输入或飞行状态无效", "approach timeout":"接近墙面超时",
  "flip distance reached":"已达到翻转距离", "flip sensor or flight failure":"翻转时雷达或飞行状态无效",
  "flip complete or contact seen":"翻转完成，正在确认接触", "wall contact confirmed":"墙面接触已确认",
  "wall capture timeout":"确认墙面接触超时", "wall contact or safety lost":"墙面接触丢失或输入无效",
  "capture safety failure":"确认接触时输入或飞行状态无效", "cancel or wall mode released":"贴墙请求已取消或释放",
  "control released":"贴墙控制已释放", "sensor health unavailable":"未收到雷达健康状态"
  , "detach requested":"离墙请求已接受", "detach rotation complete":"离墙旋转完成，正在恢复水平和高度"
  , "recovery complete":"离墙恢复完成，等待交还遥控器", "detach safety input lost":"离墙反馈无效，动作中止"
  , "recovery sensor or flight failure":"恢复阶段反馈或飞行状态无效"
  , "kill switch active or unavailable":"急停开关已生效或反馈无效，动作已禁用"
  , "bench hover: waiting for valid front/top ranges and wall distance":"台架悬停等待：两路雷达恢复有效，且前向距离满足接近条件后继续"
};
function wallView(s,busy=false){
  const live=!!s.ssh?.connected&&!!s.session?.feedback_live,control=live&&!!s.session?.control;
  const wall=fresh(s,"wall"),phase=fresh(s,"phase"),state=fresh(s,"state"),owner=fresh(s,"owner"),operator=fresh(s,"operator"),kill=fresh(s,"kill");
  const complete=["runtime","state","odom","estimator","operator","phase","owner","kill","wall"].every(key=>fresh(s,key)!==null);
  const ready=control&&complete&&!busy&&!!kill?.valid&&!kill.active;
  const names={IDLE:"贴墙待命",FRONT_WALL_DETECT:"检测墙面",STABILIZE_HOVER:"稳定悬停",SLOW_APPROACH:"缓慢接近墙面",FLIP_TO_WALL:"翻转贴墙",WALL_CAPTURE:"确认墙面接触",WALL_HOLD:"贴墙保持",WALL_PIN:"贴墙固定",DETACH_ROTATE:"离墙旋转",RECOVER:"恢复水平",EXIT:"动作结束",ABORT:"贴墙动作中止"};
  const steps={IDLE:0,FRONT_WALL_DETECT:3,STABILIZE_HOVER:3,SLOW_APPROACH:3,FLIP_TO_WALL:4,WALL_CAPTURE:4,WALL_HOLD:5,WALL_PIN:5,DETACH_ROTATE:6,RECOVER:7,EXIT:7};
  let title=wall?names[wall.state_name]||wall.state_name:"等待贴墙状态",reason=wall?.reason||"贴墙决策反馈缺失或过期",step=steps[wall?.state_name]??-1;
  const selected=owner===2||operator?.mode===3;
  if(selected&&phase==="PRESTREAM"){title="贴墙预发送";reason="建立连续姿态输出流";step=1;}
  if(selected&&phase==="ENTERING"){title="贴墙接管中";reason="等待飞控确认 OFFBOARD";step=2;}
  if(selected&&phase==="WAITING_WALL"){title="等待贴墙入口条件";reason=fresh(s,"reason")||"等待距离与稳定条件";step=0;}
  const wallPilotMode=fresh(s,"runtime")?.wall_pilot_mode||"ALTCTL";
  if(selected&&phase==="RETURNING"){title="离墙后等待遥控接管";reason="等待飞控确认 "+wallPilotMode+" 与输出释放";step=8;}
  let returned=!!(live&&wall&&(wall.recovery_completed||wall.detach_completed)&&state?.mode===wallPilotMode&&owner===0&&["IDLE","LOCKED"].includes(phase));
  if(returned){title=wall.fault_detected?"异常恢复后已交还遥控":"离墙完成，已交还遥控";reason="飞控已确认 "+wallPilotMode+" 且输出已释放，由飞手接管";step=8;}
  if(wall?.fault_detected&&!returned){title=names.ABORT;reason=wall.reason;step=-1;}
  if(live&&wall?.detach_range_height_enabled&&wall.detach_task_phase_name&&wall.detach_task_phase_name!=="NONE"&&phase!=="RC_TAKEOVER"){
    const task=wall.detach_task_phase_name;
    const taskNames={ROTATING_TO_RANGE_ENTRY:"阶段 1：离墙翻转",RANGE_HEIGHT_CONTROL:"阶段 2：测距高度控制",HANDOVER_READY:"阶段 3：准备交回 POSCTL",FAULT:"离墙异常"};
    title=taskNames[task]||title;reason=wall.detach_range_block_reason||wall.reason;
    if(task==="COMPLETED"){
      returned=!!(state?.connected&&state.mode==="POSCTL"&&owner===0&&["IDLE","LOCKED"].includes(phase));
      title=returned?(wall.detach_range_manual_recovery?"人工定高恢复完成，已交回 POSCTL":"离墙完成，已交回 POSCTL"):"等待 POSCTL 及输出释放确认";
      reason=returned?(wall.detach_range_manual_recovery?"异常后的人工恢复已完成，原故障记录保留":"飞控已确认 POSCTL，ROS 输出已释放"):"完成状态仍需新鲜飞控模式与输出归属反馈";
    }
    step=returned?8:task==="ROTATING_TO_RANGE_ENTRY"?6:task==="FAULT"?-1:7;
  }
  if(kill?.active){title="急停已生效";reason="动作禁用";step=-1;}
  if(!live){title=s.session?.push===false?"贴墙反馈已暂停":"等待贴墙实时反馈";reason="当前无法确认贴墙状态，动作禁用";step=-1;}
  const blockers=[...(s.blockers||[])],detachBlockers=[...(s.detach_blockers||[])];
  if(!live){blockers.unshift("需要有效实时反馈");detachBlockers.unshift("需要有效实时反馈");}
  else if(!control){blockers.unshift("请先申请控制权");detachBlockers.unshift("请先申请控制权");}
  else if(!complete){blockers.unshift("贴墙所需反馈缺失或过期");detachBlockers.unshift("离墙所需反馈缺失或过期");}
  return {live,wall,phase,selected,title,reason:wallTranslations[reason]||reason,step,returned,blockers,detachBlockers,
    canWall:ready&&!!s.can_wall,canDetach:ready&&!!fresh(s,"setpoint")&&!!s.can_detach,
    canCancel:control&&!busy&&!!s.can_cancel_wait,
    holding:!!(live&&complete&&kill?.valid&&!kill.active&&s.success&&wall?.state_name==="WALL_HOLD"&&state?.mode==="OFFBOARD"&&phase==="ACTIVE"&&owner===2),
    holdAchieved:!!s.status?.wall?.value?.hold_achieved};
}
function rangeView(s,busy=false){
  const wall=fresh(s,"wall"),flight=fresh(s,"state"),kill=fresh(s,"kill"),phase=fresh(s,"phase");
  const active=!!(s.ssh?.connected&&s.session?.feedback_live&&s.session?.push!==false&&wall?.detach_range_height_enabled&&wall.detach_task_phase_name&&wall.detach_task_phase_name!=="NONE");
  const canSetHeight=!!(s.ssh?.connected&&s.session?.feedback_live&&s.session?.push!==false&&s.session?.control&&!busy&&
    kill?.valid&&!kill.active&&flight?.connected&&flight.armed&&flight.mode==="OFFBOARD"&&fresh(s,"owner")===2&&fresh(s,"setpoint")&&
    ["ACTIVE","RETURNING"].includes(phase)&&s.can_set_range_height&&wall?.detach_range_manual_available);
  return {active,wall,canSetHeight,target:wall?.detach_range_manual_target_m??s.config?.detach_range_manual_target_m??1.5,
    reason:canSetHeight?"可提交固定目标；保留实测高度和首帧证据":s.set_range_height_blockers?.[0]||wall?.detach_range_manual_block_reason||"等待离墙回平、有效测距及控制权限"};
}
if(typeof module!=="undefined")module.exports={view,fresh,wallView,rangeView,maintenanceView};
if(typeof document!=="undefined"){
const $=id=>document.getElementById(id),form=$("config");
let latest=null,token="",busy=false,recoveryBusy=false,loaded=false,polling=false,wasDetaching=false;
const text=(id,value)=>{$(id).textContent=value;};
const n=(v,d=2)=>typeof v==="number"&&Number.isFinite(v)?v.toFixed(d):"—";
const summarize=items=>items.slice(0,2).join("；")+(items.length>2?"（另有 "+(items.length-2)+" 项，见贴墙就绪检查）":"");
const tell=(v,error=false)=>{text("message",v);$("message").classList.toggle("error",error);};
function paint(s){
  latest=s;token=s.token||"";const v=view(s,busy||recoveryBusy),w=wallView(s,busy||recoveryBusy),paused=s.session?.push===false,m=maintenanceView(s,busy,recoveryBusy);
  $("dot").classList.toggle("live",v.live);text("connection",paused?"实时反馈已暂停":v.live?"实时反馈已连接":"实时反馈未连接");text("host",s.ssh?.host||"树莓派 ROS1 · SSH");
  text("session",(s.session?.control?"本机为控制端":s.session?.owner?"本机为观察端 · 当前控制端："+s.session.owner:"本机为观察端 · 申请控制权后可操作")+" · 在线 "+(s.session?.clients||0)+" 台"+(paused?" · 反馈暂停中":""));
  text("control",s.session?.control?"释放控制权":"申请控制权");$("control").disabled=busy||!s.ssh?.connected||(!s.session?.control&&!!s.session?.owner);
  text("push",paused?"恢复实时反馈":"暂停实时反馈");$("push").disabled=busy||!s.ssh?.connected;
  $("repairMavros").disabled=!m.canRepair;$("repairMavros").title=m.repairReason;
  text("repairMavros",s.mavros_repair?.state==="RUNNING"?"正在重启 MAVROS…":"重启 MAVROS（地面恢复）");
  text("repairStatus",m.repairReason||(s.mavros_repair&&s.mavros_repair.state!=="IDLE"?s.mavros_repair.reason:"先确认落地、上锁且电机停止；自动清理旧控制、雷达和日志，再重启 MAVROS。成功后手动启动基础链路。"));
  $("start").disabled=!v.canStart;$("attach").disabled=!v.canAttach;$("detach").disabled=!v.canDetach;
  $("stop").disabled=!m.canStop;$("stop").title=m.stopReason;text("stopStatus",m.stopReason||"落地上锁后可停止；飞控反馈过期时需人工确认电机已停止。");
  $("wallAction").disabled=!w.canWall;$("wallDetach").disabled=!w.canDetach;
  $("cancelWait").hidden=!s.can_cancel_wait;$("cancelWait").disabled=!w.canCancel;
  text("wallBlockReason",w.canWall?"贴墙入口已就绪":summarize(w.blockers)||"等待贴墙入口条件");
  text("wallDetachReason",w.canDetach?"可提交离墙请求，控制节点确认连续稳定后执行":summarize(w.detachBlockers)||"等待当前贴墙保持与离墙条件");
  text("start",s.starting?(s.mavros_repair?.state==="RUNNING"?"正在修复 MAVROS…":"正在配置并启动…"):v.enabled?"基础链路已启动":"一键配置并启动");
  text("configStatus",s.launch_error||(!v.live?"反馈未连接，无法确认配置":v.enabled?"正在使用现场配置，参数已锁定":s.starting?"启动请求已提交，等待实际反馈":"修改下方参数后点击应用"));
  text("attachReason",v.canAttach?"入口已就绪，点击后等待节点回执":v.reasons[0]||"等待顶吸节点就绪");text("detachReason",v.detachReason);
  if(!loaded||v.enabled){for(const field of form.elements)if(field.name in (s.config||{}))field.value=s.config[field.name];loaded=true;}
  for(const field of form.elements)field.disabled=!v.editable;
  text("source",v.enabled?"现场配置":"启动配置");
  let stage=v.stage,reason=v.reason;
  if(v.live&&v.ceiling?.state_name==="DETACH")wasDetaching=true;
  if(wasDetaching&&v.live&&v.ceiling?.state_name==="NORMAL_FLIGHT"&&v.state?.mode==="ALTCTL"&&v.owner===0&&v.phase==="IDLE"){
    stage="已交还遥控";reason="飞控已确认 ALTCTL 且输出归属已释放，由飞手接管降落";
  }
  if(w.selected||w.returned){stage=w.title;reason=w.reason;}
  if(v.live&&v.phase==="RC_TAKEOVER"){stage="遥控接管中";reason=fresh(s,"rc_takeover")||fresh(s,"reason")||"等待接管反馈和原遥控模式确认";}
  if(s.mavros_repair?.state==="RUNNING"){stage="MAVROS 连接修复中";reason=s.mavros_repair.reason;}
  if(paused){stage="实时反馈已暂停";reason="仅暂停本设备的反馈；飞行程序和日志继续运行。恢复反馈后才能发送动作。";}
  text("stage",stage);text("reason",reason);text("phase",v.live?v.phase||"等待节点":"离线");
  text("receipt","顶吸回执："+(s.ceiling_request?.reason||"尚未发送")+(!v.live?"（历史回执）":""));
  document.querySelectorAll("#ceilingSteps li").forEach((el,i)=>el.classList.toggle("current",v.live&&i===v.step));
  const hw=w.wall;
  const range=rangeView(s,busy),r=range.wall;
  $("setRangeHeight").disabled=!range.canSetHeight;
  text("setRangeHeight","一键定高至 "+n(range.target)+" m");text("setRangeHeightReason",range.reason);
  text("rangeHeightState",range.active?"本轮 "+r.detach_range_episode+" · 原始距离 "+n(r.detach_range_raw_m)+" m · 测距高度 "+n(r.detach_range_height_m)+" m · 目标 "+(r.detach_range_target_locked?n(r.detach_range_target_m)+" m":"等待首帧")+" · "+(r.detach_range_target_source==="manual_fixed_range_height"?"人工固定目标":"首条合格测距目标")+" · "+(r.detach_range_valid?"反馈有效":"反馈无效"):"测距离墙任务：未开始或反馈过期");
  text("rangeHeightEvidence",range.active?"首帧原值 / 几何高度 / 倾角："+n(r.detach_range_lock_raw_m)+" m / "+n(r.detach_range_lock_height_m)+" m / "+n(r.detach_range_lock_tilt_deg)+"°；源 / 接收时间："+n(r.detach_range_lock_source_time,3)+" / "+n(r.detach_range_lock_received_time,3)+(r.detach_range_manual_recovery?"；异常后人工恢复，原故障："+r.detach_range_previous_fault:""):"等待首帧证据；原相对起飞高度仅作融合诊断");
  text("rangeHeightGate",range.active?"测距速度 "+(r.detach_range_velocity_valid?n(r.detach_range_velocity_mps)+" m/s":"未就绪")+" · 反馈年龄 "+n(r.detach_range_age_s)+" s · 连续稳定 "+n(r.detach_range_stable_elapsed_s)+" s / "+r.detach_range_stable_samples+" 条新测距 · "+(r.detach_range_block_reason||"等待条件"):"等待测距及交回条件");
  const fusion=v.live?fresh(s,"wall_sensor_fusion"):null,fusionNames={READY:"参数就绪",SUSPENDED:"融合已暂停",RESTORING:"请求恢复中",SETTLING:"等待应用缓冲与确认",ERROR:"异常",DISABLING:"请求暂停中",INITIALIZING:"读取原值",DISABLED:"未启用"};
  text("wallFusionState",fusion?"光流 / 测距融合："+(fusionNames[fusion.phase]||fusion.phase)+" · 实测倾角 "+n(fusion.measured_tilt_deg)+"° · "+(fusion.control_ready?"恢复条件已确认":"等待确认"):"光流 / 测距融合：无新鲜反馈");
  text("wallFusionReason",fusion?(fusion.reason||"")+"；参数原值 / 目标 / 确认：光流 "+[fusion.original_flow,fusion.target_flow,fusion.confirmed_flow].join(" / ")+"；测距 "+[fusion.original_range,fusion.target_range,fusion.confirmed_range].join(" / ")+"。80°请求恢复；实际融合需核对飞控记录。":"等待融合参数状态");
  $("offlinePanel").hidden=!s.offline;
  if(s.offline){
    const select=$("mockScenario");
    if(!select.options.length)for(const [value,label] of Object.entries(s.mock_scenarios||{})){const option=document.createElement("option");option.value=value;option.textContent=label;select.append(option);}
    select.value=s.mock_scenario||"grounded";select.disabled=busy||!s.session?.control||s.session?.push===false;
  }
  const hs=hw?.horizontal_velocity_enabled&&Number.isFinite(hw.horizontal_velocity_x_enu_mps)&&Number.isFinite(hw.horizontal_velocity_y_enu_mps)?Math.hypot(hw.horizontal_velocity_x_enu_mps,hw.horizontal_velocity_y_enu_mps):NaN;
  text("wallHorizontalState",hw?.horizontal_velocity_enabled?"水平速度闭环："+(hw.horizontal_velocity_reason||"等待反馈")+" · 水平速度 "+(Number.isFinite(hs)?hs.toFixed(3):"—")+" m/s · 光流质量 "+hw.optical_flow_quality+(hw.horizontal_velocity_stable?" · 稳定确认完成":""):"水平速度闭环未启用或反馈过期");
  text("wallStage",w.title);text("wallReason",w.reason);text("wallPhase",w.live?w.wall?.state_name||"等待节点":"未确认");
  document.querySelectorAll(".wallSteps li").forEach((el,i)=>{el.classList.toggle("current",w.live&&i===w.step);el.classList.toggle("done",w.live&&i<w.step);});
  text("wallMilestone",w.holding?"控制节点已确认 WALL_HOLD，当前仍在贴墙保持。":w.holdAchieved?"本轮曾成功进入贴墙保持；历史记录不代表当前仍在保持。":"本轮尚未确认贴墙保持");
  $("wallMilestone").classList.toggle("confirmed",w.holding);
  text("wallDetachCondition",!w.live?"实时反馈未确认，不能判断离墙条件":w.wall?.state_name!=="WALL_HOLD"?"离墙请求仅在当前贴墙保持时接受":w.wall.detach_pending?"离墙请求等待中："+(w.wall.detach_block_reason||"等待连续稳定"):w.wall.detach_ready?"离墙旋转条件已满足":"旋转前还需："+(w.wall.detach_block_reason||"等待姿态和角速度连续稳定"));
  const c=v.live?v.ceiling:null,odom=v.live?fresh(s,"odom"):null,state=v.live?v.state:null;
  text("distance",c?.range_fresh?n(c.ceiling_distance_filtered_m,3):"—");
  const output=v.live?fresh(s,"setpoint"):null;text("thrust",output?n(output.thrust*100,1):"—");
  text("descent",c&&(c.height_source!=="barometer_hold_relative"||(c.baro_fresh&&c.baro_reference_ready))?n(c.descended_height_m,3):"—");
  text("velocityLabel",range.active?"测距竖直速度":"融合竖直速度");
  text("velocity",range.active?(r.detach_range_velocity_valid?n(r.detach_range_velocity_mps):"未就绪"):n(odom?.vertical_speed));
  const health=v.live?fresh(s,"health"):null;
  text("front",v.live&&v.runtime?.front_sensor_masked?"已屏蔽":health?.front_valid?n(fresh(s,"front")?.range_m,3):"—");
  text("height",v.live?n(w.wall?.height_above_takeoff_m):"—");
  text("rangeWarning",v.live?(s.sensor_warnings||[]).join("；"):"距离反馈当前无法确认");
  const angles=odom?.angles;
  text("pitch",n(angles?.[1],1));text("roll",n(angles?.[0],1));text("yaw",n(angles?.[2],1));
  text("mode",state?.mode||"—");text("armed",state?(state.armed?"已解锁":"未解锁"):"—");
  text("battery",v.live?n(fresh(s,"battery")?.voltage,1):"—");text("owner",v.live?({0:"无",1:"顶吸",2:"侧吸"}[v.owner]||"—"):"—");
  text("baro",!c?"—":c.baro_fresh&&c.baro_reference_ready?"有效":c.height_source==="barometer_hold_relative"?"未就绪":"当前未使用");
  text("kill",!v.live||!v.kill?.valid?"急停配置或反馈无效／过期":v.kill.active?"急停已触发 · 第 "+v.kill.channel+" 通道":v.kill.channel?"急停未触发 · 第 "+v.kill.channel+" 通道":"飞控未配置遥控急停通道");
  text("recording",v.live&&v.runtime?v.runtime.logger?"日志录制中":"日志未录制":"日志状态未知");
  const reasons=v.canAttach?["顶吸入口已就绪"]:v.reasons.length?v.reasons:["等待顶吸入口条件满足"];
  $("checks").replaceChildren(...reasons.map(reason=>{const el=document.createElement("div");el.className="check"+(v.canAttach?" ok":"");el.textContent=reason;return el;}));
  $("wallChecks").replaceChildren(...(s.checks||[]).map(check=>{let label=check.label,ok=check.ok,value=ok?"就绪":check.reason||"待满足";if(w.live&&w.selected&&w.phase==="ACTIVE"&&v.state?.mode==="OFFBOARD"&&["mode","idle"].includes(check.id)){label=check.id==="mode"?"OFFBOARD 模式":"贴墙动作";ok=true;value="执行中";}const el=document.createElement("div");el.className="check"+(w.live&&ok?" ok":"");el.textContent=label+" · "+(w.live?value:"待确认");return el;}));
  const ages=[["飞控心跳","state"],["里程计 / 姿态","odom"],["估计器","estimator"],["遥控输入","rc"],["急停","kill"],["雷达健康","health"],["顶部距离","ceiling"],["前向距离","front"],["顶吸状态","ceiling_status"],["贴墙状态","wall"],["实际设定值","setpoint"],["基础程序","runtime"]];
  $("ages").replaceChildren(...ages.map(([label,key])=>{const item=s.status?.[key],el=document.createElement("div");el.className="check"+(v.live&&item?.fresh?" ok":"");el.textContent=label+" · "+(paused?"已暂停":!v.live?"连接中断":item?.age_s==null?"未收到":n(item.age_s,2)+" s · "+(item.fresh?"有效":"已过期"));return el;}));
  text("bridgeError",s.ssh?.error||s.launch_error||"");
  if(s.events?.length)$("events").replaceChildren(...s.events.slice(-18).reverse().map(event=>{const el=document.createElement("div"),time=document.createElement("time"),body=document.createElement("span");el.className="event";time.textContent=new Date(event.time*1000).toLocaleTimeString("zh-CN",{hour12:false});body.textContent=event.text;el.append(time,body);return el;}));
}
async function action(path,data={},recovery=false){
  if(recovery?recoveryBusy:(busy||recoveryBusy))return;
  if(recovery)recoveryBusy=true;else busy=true;
  try{
    if(latest)paint(latest);
    tell(path==="/api/start"?"配置并启动请求已提交，等待回执…":"请求已提交，等待回执…");
    const response=await fetch(path,{method:"POST",headers:{"Content-Type":"application/json","X-Console-Token":token},body:JSON.stringify(data),signal:AbortSignal.timeout(path==="/api/stop"?20000:10000)});
    const result=await response.json();if(response.status===401){window.location.replace("/login");return;}if(!response.ok)throw new Error(result.error||"请求失败");tell(result.message||"请求已发送，请观察节点执行状态");
  }catch(error){tell(error.name==="TimeoutError"?"回执超时，执行结果未知；请核对实时状态，页面不会自动重发":error.message,true);}
  finally{if(recovery)recoveryBusy=false;else busy=false;await poll();}
}
$("control").addEventListener("click",()=>action(latest?.session?.control?"/api/release":"/api/acquire"));
$("push").addEventListener("click",()=>action("/api/push",{enabled:latest?.session?.push===false}));
$("wallAction").addEventListener("click",()=>{if(latest&&wallView(latest,busy).canWall)action("/api/wall");});
$("wallDetach").addEventListener("click",()=>{if(latest&&wallView(latest,busy).canDetach)action("/api/detach");});
$("setRangeHeight").addEventListener("click",()=>{if(latest&&rangeView(latest,busy).canSetHeight)action("/api/set-range-height");});
$("mockScenario").addEventListener("change",()=>action("/api/mock-scenario",{scenario:$("mockScenario").value}));
$("cancelWait").addEventListener("click",()=>{if(latest&&wallView(latest,busy).canCancel)action("/api/cancel-wait");});
$("repairMavros").addEventListener("click",()=>{if(!latest||!maintenanceView(latest,busy,recoveryBusy).canRepair)return;if(confirm("确认飞机已落地、上锁且电机已停止？反馈过期时必须现场核实。将停止控制、雷达和日志，清理旧链路并重启 MAVROS；成功后需手动启动基础链路。"))action("/api/repair-mavros",{grounded_confirmed:true},true);});
$("start").addEventListener("click",()=>{if(!latest||!view(latest,busy).canStart||!form.reportValidity())return;const data=Object.fromEntries(new FormData(form));data.baud=Number(data.baud);data.hover_thrust=Number(data.hover_thrust);data.detach_range_manual_target_m=Number(data.detach_range_manual_target_m);action("/api/start",data);});
$("attach").addEventListener("click",()=>{if(latest&&view(latest,busy).canAttach){wasDetaching=false;action("/api/ceiling");}});
$("detach").addEventListener("click",()=>{if(latest&&view(latest,busy).canDetach)action("/api/ceiling-detach");});
$("stop").addEventListener("click",()=>{if(confirm("确认飞机已落地且电机已停止？将停止控制、MAVROS、雷达和日志。"))action("/api/stop",{grounded_confirmed:true});});
form.addEventListener("submit",event=>event.preventDefault());
async function poll(){
  if(polling)return;polling=true;
  try{const response=await fetch("/api/status",{cache:"no-store",signal:AbortSignal.timeout(2500)});if(response.status===401){window.location.replace("/login");return;}if(!response.ok)throw new Error("本机服务响应异常");paint(await response.json());}
  catch(error){paint({...latest,ssh:{connected:false,error:"测试台服务无响应："+error.message},session:{control:false,feedback_live:false},can_ceiling:false,can_ceiling_detach:false});}
  finally{polling=false;}
}
setInterval(poll,300);poll();
}
