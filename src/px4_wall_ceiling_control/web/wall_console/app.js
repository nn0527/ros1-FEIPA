"use strict";
const $ = id => document.getElementById(id);
let token = "", loaded = false, requestBusy = false, latest = null;
const form = $("config");
const translate = {
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
const wallNames = {IDLE:"等待贴墙指令", FRONT_WALL_DETECT:"检测墙面", STABILIZE_HOVER:"稳定悬停",
  SLOW_APPROACH:"缓慢接近墙面", FLIP_TO_WALL:"翻转贴墙", WALL_CAPTURE:"确认墙面接触",
  WALL_HOLD:"贴墙保持", WALL_PIN:"贴墙固定", DETACH_ROTATE:"脱离旋转", RECOVER:"恢复飞行",
  EXIT:"动作结束", ABORT:"贴墙动作已中止"};
function text(id, value){$(id).textContent = value;}
function n(value, digits=2){return typeof value === "number" && Number.isFinite(value) ? value.toFixed(digits) : "—";}
function fresh(snapshot, name){const item=snapshot.status[name];return item && item.fresh ? item.value : null;}
function message(value, error=false){text("actionMessage",value);$("actionMessage").classList.toggle("error",error);}
function rows(container, data, make){container.replaceChildren(...data.map(make));}
function pair(label, value, className=""){
  const el=document.createElement("div");el.className=className;
  const a=document.createElement("span"),b=document.createElement("span");a.textContent=label;b.textContent=value;
  el.append(a,b);return el;
}
function paint(s){
  latest=s; token=s.token;
  const ceilingStatus=fresh(s,"ceiling_status");
  const request=s.ceiling_request;
  text("ceilingRequestState", "吸顶请求："+(request?.reason || "尚未发送"));
  $("ceilingAction").disabled=requestBusy || !s.session?.feedback_live || !s.session?.control || !s.can_ceiling;
  $("ceilingDetach").disabled=requestBusy || !s.session?.feedback_live || !s.session?.control || !s.can_ceiling_detach;
  text("ceilingBlockReason",s.can_ceiling?"吸顶入口条件已满足":(s.ceiling_blockers || ["等待吸顶服务升级"]).join("；"));
  const ceilingNames={NORMAL_FLIGHT:"待命",CEILING_ARMED:"吸顶准备",APPROACH:"向上接近",ATTACH_CONTROL:"接触控制",SURFACE_HOLD:"吸顶保持",DETACH:"脱离顶部",RECOVERY_HOVER:"恢复悬停",FAULT:"故障"};
  text("ceilingStage",ceilingStatus?"吸顶："+(ceilingNames[ceilingStatus.state_name]||ceilingStatus.state_name)+" · "+ceilingStatus.reason+" · 顶距 "+n(ceilingStatus.ceiling_distance_filtered_m,3)+" m"+(fresh(s,"runtime")?.front_sensor_masked?" · 前部雷达已屏蔽（ALTCTL接管后恢复）":"")+(typeof ceilingStatus.descended_height_m === "number"?" · 已下降 "+n(ceilingStatus.descended_height_m*100,1)+" / 50 cm":""):"吸顶：无新鲜状态反馈");
  if(!loaded || fresh(s,"runtime")?.enabled){for(const input of form.elements) if(input.name in s.config) input.value=s.config[input.name];loaded=true;}
  const ssh=s.ssh.connected, session=s.session || {}, live=!!session.feedback_live, control=!!session.control, state=fresh(s,"state"), runtime=fresh(s,"runtime"), phase=fresh(s,"phase"), wall=fresh(s,"wall"), health=fresh(s,"health"), odom=fresh(s,"odom");
  text("connectionText",ssh?"SSH 已连接":"SSH 未连接");text("host",s.ssh.host);
  $("connectionDot").classList.toggle("ok",ssh);
  text("sessionText",(control?"本机为控制端":session.owner?"本机为观察端 · 控制端："+session.owner:"本机为观察端 · 暂无控制端")+" · 在线 "+(session.clients || 0)+" 台 · "+(!session.push?"实时反馈已暂停":live?"实时反馈已开启":"等待新反馈"));
  text("control",control?"释放控制权":"申请控制权");
  $("control").disabled=requestBusy || !ssh || (!control && !!session.owner);
  text("push",session.push?"暂停实时反馈":"开启实时反馈");
  $("push").disabled=requestBusy || !ssh;
  text("flightConnection",state ? (state.connected?"已连接":"未连接") : "无有效反馈");
  text("armed",state ? (state.armed?"已解锁":"未解锁") : "—");text("mode",state?.mode || "—");
  const enabled=!!runtime?.enabled;
  if(runtime?.front_sensor_masked)text("front","已屏蔽");
  $("start").disabled=requestBusy || !live || !control || s.starting || (state?.armed && !enabled);
  text("startLabel",s.starting?"正在启动…":enabled?"基础程序已启动":"启动基础程序");
  $("wall").disabled=requestBusy || !live || !control || !s.can_wall;
  $("detach").disabled=requestBusy || !live || !control || !s.can_detach;
  $("stop").disabled=requestBusy || !ssh || !control;
  text("detachBlockReason",s.can_detach?"可提交离墙请求；控制节点确认连续稳定后才旋转": "离墙："+(s.detach_blockers?.[0] || "等待贴墙保持"));
  $("detachBlockReason").classList.toggle("ready",!!s.can_detach);
  const kill=fresh(s,"kill");
  text("killState",!kill?.valid?"急停：配置或通道反馈无效，动作禁用":kill.active?"急停已生效 · 第 "+kill.channel+" 通道 · 动作禁用":kill.channel?"急停未触发 · 第 "+kill.channel+" 通道":"飞控未配置遥控急停通道");
  $("killState").classList.toggle("errorText",!kill?.valid || !!kill.active);
  text("detachCondition",wall?.state_name!=="WALL_HOLD"?"离墙仅在当前贴墙保持阶段接受请求":wall.detach_pending?"离墙请求等待中："+(wall.detach_block_reason || "等待连续稳定"):wall.detach_ready?"旋转条件已满足，可提交离墙请求":"可提交离墙请求，旋转前还需："+(wall.detach_block_reason || "等待姿态和角速度稳定"));
  text("wallBlockReason",s.can_wall?"":s.blockers.join("；"));
  for(const field of form.elements) field.disabled=s.starting || enabled || requestBusy || !control || !live;
  let title="等待启动", reason=s.blockers[0] || "飞行稳定后，可发送贴墙指令", step=0;
  if(!ssh){title="SSH 反馈中断";reason=s.ssh.error || "正在重新连接树莓派";}
  else if(s.starting){title="基础程序启动中";reason="正在启动 MAVROS、雷达与贴墙控制节点";}
  else if(s.launch_error){title="启动未完成";reason=s.launch_error;}
  else if(phase==="PRESTREAM"){title="预发送姿态目标";reason="建立连续设定值流，等待模式交接";step=1;}
  else if(phase==="ENTERING"){title="进入 OFFBOARD";reason="等待飞控确认模式切换";step=2;}
  else if(phase==="ACTIVE"){
    title=fresh(s,"operator")?.mode===2 ? "吸顶："+(ceilingNames[ceilingStatus?.state_name]||"等待反馈") : wallNames[wall?.state_name] || "贴墙控制中";reason=fresh(s,"operator")?.mode===2 ? ceilingStatus?.reason || "等待吸顶状态反馈" : wall?.reason || "等待贴墙状态反馈";
    step=wall?.state_name==="WALL_HOLD"?5:["FLIP_TO_WALL","WALL_CAPTURE","WALL_PIN"].includes(wall?.state_name)?4:3;
  } else if(phase==="LOCKED"){title="动作受阻";reason=fresh(s,"reason") || reason;}
  else if(phase && phase!=="IDLE"){title=phase;reason=fresh(s,"reason") || reason;}
  else if(enabled){title=s.pending?"贴墙请求已发送":s.can_wall?"可以发送贴墙指令":"等待稳定飞行";}
  if(wall?.state_name==="ABORT"){title=wallNames.ABORT;reason=wall.reason;}
  if(s.success){title="贴墙保持";reason="控制节点已确认接触，当前保持贴墙";step=5;}
  const history=s.status.wall?.value;
  const reached={SLOW_APPROACH:3,FLIP_TO_WALL:4,WALL_CAPTURE:4,WALL_HOLD:5,WALL_PIN:4,DETACH_ROTATE:6,RECOVER:7,EXIT:7};
  step=Math.max(step,reached[history?.reached_state_name] || 0,history?.hold_achieved?5:0);
  const recovered=history?.recovery_completed || history?.detach_completed;
  const returned=!!(recovered && state?.mode==="ALTCTL" && fresh(s,"owner")===0 && ["IDLE","LOCKED"].includes(phase));
  if(recovered)step=8;
  if(phase==="RETURNING"){title="等待遥控接管";reason="保持水平控制输出，等待飞控确认 ALTCTL；未确认前不结束交接";}
  if(returned){title=history?.fault_detected?"异常恢复完成，已交还遥控":"离墙完成，已交还遥控";reason="已确认 ALTCTL，输出已释放；飞手降低油门落地，再按正常流程上锁";}
  if(kill?.active){title="急停已生效";reason=state?.armed?"电机已停，飞控等待自动上锁；动作请求已禁用":"飞控已上锁，急停开关仍生效";}
  if(!live){title=!session.push?"实时反馈已暂停":"等待实时反馈";reason=!session.push?"本机停止接收数据；飞行程序与记录继续运行，动作已禁用":"需要收到新反馈后才能操作";}
  text("stageTitle",title);text("stageReason",translate[reason] || reason);
  document.querySelector(".stage").classList.toggle("error",!!kill?.active || !!s.launch_error || phase==="LOCKED" || wall?.fault_detected || wall?.state_name==="ABORT");
  $("successNote").hidden=!history?.hold_achieved || !live;
  text("successNote",s.success?"本轮已确认贴墙保持；当前仍在保持。":history?.hold_achieved?"本轮曾成功进入贴墙保持，记录已保留。当前状态见上方；历史结果不代表现在仍贴墙。":"");
  document.querySelectorAll(".steps li").forEach((el,i)=>{el.classList.toggle("done",i<step || (history?.hold_achieved && i===5) || (returned && i===step));el.classList.toggle("current",live && !returned && wall?.state_name!=="ABORT" && i===step);});
  text("front",health?.front_valid?n(fresh(s,"front")?.range_m):"—");
  text("ceiling",health?.ceiling_valid?n(fresh(s,"ceiling")?.range_m):"—");
  text("angles",odom?.angles ? n(odom.angles[0],1)+" / "+n(odom.angles[1],1) : "—");
  const thrust=fresh(s,"setpoint")?.thrust;text("thrust",typeof thrust==="number"?n(thrust*100,1):"—");
  text("velocity",n(odom?.vertical_speed));text("battery",n(fresh(s,"battery")?.voltage,1));
  text("height",n(wall?.height_above_takeoff_m));
  if(runtime?.front_sensor_masked)text("front","已屏蔽");
  text("rangeWarning",(s.sensor_warnings || []).join("；") +
    (s.config.test_profile==="restrained_bench"?"　有约束台架：高度和雷达入口门槛关闭；距离恢复前悬停等待，接近／翻转仍需有效雷达。":""));
  const owner=fresh(s,"owner");text("owner",owner===2?"侧吸":owner===1?"吸顶":owner===0?"无":"—");
  rows($("checks"),s.checks,check=>{
    let label=check.label,ok=check.ok,value=ok?"就绪":"待满足";
    if(phase==="ACTIVE" && state?.mode==="OFFBOARD" && check.id==="mode"){label="OFFBOARD 模式";ok=true;value="控制中";}
    if(phase==="ACTIVE" && check.id==="idle"){label="贴墙动作";ok=true;value="执行中";}
    const el=pair(label,value,"check"+(ok?" ok":""));el.title=check.reason;return el;
  });
  const ages=[["飞控心跳","state"],["里程计","odom"],["估计器","estimator"],["遥控输入","rc"],["急停反馈","kill"],["两路雷达","health"],["贴墙状态","wall"]];
  rows($("ages"),ages,([label,key])=>{const item=s.status[key],el=pair(label,!session.push?"推送已暂停":item.age_s===null?"未收到":n(item.age_s,2)+" s","ageRow");el.lastChild.classList.toggle("stale",!item.fresh);return el;});
  text("sshError",ssh?"":s.ssh.error);
  text("recording",!live?"试飞日志：当前无法确认":runtime?.logger?"试飞日志：录制中":enabled?"试飞日志：未检测到录制节点":"试飞日志：等待启动");
  if(s.events.length){rows($("events"),s.events.slice(-25).reverse(),event=>pair(new Date(event.time*1000).toLocaleTimeString("zh-CN",{hour12:false}),event.text,"eventRow "+event.kind));}
}
$("control").addEventListener("click",()=>action(latest?.session?.control?"/api/release":"/api/acquire",{}));
$("push").addEventListener("click",()=>action("/api/push",{enabled:!latest?.session?.push}));
async function action(path, data){
  if(requestBusy)return;requestBusy=true;$("ceilingAction").disabled=true;$("ceilingDetach").disabled=true;$("stop").disabled=true;$("start").disabled=true;$("wall").disabled=true;$("detach").disabled=true;
  message(path==="/api/stop"?"正在终止现有进程…":path==="/api/wall"?"正在发送 wall…":path==="/api/detach"?"正在发送 detach…":path==="/api/start"?"正在请求启动…":path==="/api/ceiling"?"正在发送吸顶请求并等待节点回执…":path==="/api/ceiling-detach"?"正在发送离顶请求…":"正在更新连接设置…");
  try{
    const response=await fetch(path,{method:"POST",headers:{"Content-Type":"application/json","X-Console-Token":token},body:JSON.stringify(data),signal:AbortSignal.timeout(9000)});
    const result=await response.json();if(!response.ok)throw new Error(result.error || "操作失败");message(result.message);
  }catch(error){message(error.name==="TimeoutError"?"操作确认超时；请核对状态，页面不会自动重发":error.message,true);}
  finally{requestBusy=false;await poll();}
}
$("start").addEventListener("click",()=>{
  if(!form.reportValidity())return;
  const data=Object.fromEntries(new FormData(form));
  if(latest && fresh(latest,"runtime")?.enabled){action("/api/start",latest.config);return;}
  data.baud=Number(data.baud);data.hover_thrust=Number(data.hover_thrust);action("/api/start",data);
});
$("ceilingAction").addEventListener("click",()=>action("/api/ceiling",{}));
$("ceilingDetach").addEventListener("click",()=>action("/api/ceiling-detach",{}));
$("wall").addEventListener("click",()=>action("/api/wall",{}));
$("detach").addEventListener("click",()=>action("/api/detach",{}));
$("stop").addEventListener("click",()=>{
  if(window.confirm("确认飞机已落地且电机已停止？此操作将终止试验控制、MAVROS、雷达和日志记录进程。"))
    action("/api/stop",{grounded_confirmed:true});
});
let polling=false;
async function poll(){
  if(polling)return;polling=true;
  try{
    const response=await fetch("/api/status",{cache:"no-store",signal:AbortSignal.timeout(2500)});
    if(!response.ok)throw new Error("本机服务响应异常");paint(await response.json());
  }catch(error){
    $("ceilingAction").disabled=true;$("ceilingDetach").disabled=true;text("ceilingStage","吸顶：实时反馈中断");$("stop").disabled=true;$("control").disabled=true;$("push").disabled=true;text("connectionText","本机服务无响应");$("connectionDot").classList.remove("ok");$("wall").disabled=true;$("start").disabled=true;$("detach").disabled=true;
    if(!$("successNote").hidden)text("successNote","本轮曾成功进入贴墙保持；当前反馈中断，不能确认现在的状态。");
    text("stageTitle","反馈中断");text("stageReason","无法确认当前阶段，请核对飞控状态");
    text("killState","急停反馈中断，动作禁用");text("detachCondition","实时反馈中断，离墙禁用");
    for(const id of ["front","ceiling","angles","thrust","mode","height","velocity","battery","owner","armed","flightConnection"])text(id,"—");
    text("sshError",error.message);document.querySelectorAll(".steps li").forEach(el=>el.classList.remove("current"));
    document.querySelectorAll(".check").forEach(el=>{el.classList.remove("ok");el.lastChild.textContent="待确认";});
    document.querySelectorAll(".ageRow").forEach(el=>{el.lastChild.textContent="反馈中断";el.lastChild.classList.add("stale");});
    text("recording","试飞日志：无法确认");
  }finally{polling=false;}
}
setInterval(poll,200);poll();
