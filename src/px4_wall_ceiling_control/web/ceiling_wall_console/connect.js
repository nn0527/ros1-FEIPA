"use strict";
function consoleAddress(ip, port){
  ip=String(ip).trim();port=String(port).trim();
  const octets=ip.split(".");
  if(octets.length!==4||!octets.every(part=>/^(0|[1-9]\d{0,2})$/.test(part)&&Number(part)<=255))throw new Error("请输入有效 IPv4 地址，例如 172.20.10.2");
  if(!/^\d{1,5}$/.test(port)||Number(port)<1||Number(port)>65535)throw new Error("端口应为 1 到 65535 之间的整数");
  return "http://"+ip+":"+Number(port)+"/login";
}
if(typeof module!=="undefined")module.exports={consoleAddress};
if(typeof document!=="undefined"){
  const form=document.getElementById("connectForm"),preset=document.getElementById("consolePreset"),ip=document.getElementById("consoleIp"),port=document.getElementById("consolePort"),target=document.getElementById("targetAddress"),message=document.getElementById("connectMessage");
  let saved=null;
  try{saved=JSON.parse((localStorage.getItem("ceilingWallConsoleAddress") || localStorage.getItem("ceilingConsoleAddress"))||"null");}catch(error){}
  const current=window.location.protocol==="http:"?window.location.hostname:"";
  ip.value=current||saved?.ip||"172.20.10.2";
  port.value=(current?window.location.port:"")||saved?.port||"8790";
  function preview(){
    preset.value=["172.20.10.2","10.187.58.66","192.168.2.20","127.0.0.1"].includes(ip.value.trim())?ip.value.trim():"custom";
    try{target.textContent=consoleAddress(ip.value,port.value);message.textContent="";}
    catch(error){target.textContent="检查 IP 与端口后连接";}
  }
  preset.addEventListener("change",()=>{if(preset.value!=="custom"){ip.value=preset.value;preview();}else{ip.value="";target.textContent="请输入树莓派 IP";ip.focus();}});
  ip.addEventListener("input",preview);port.addEventListener("input",preview);
  form.addEventListener("submit",event=>{
    event.preventDefault();if(!form.reportValidity())return;
    try{
      const address=consoleAddress(ip.value,port.value);
      try{localStorage.setItem("ceilingWallConsoleAddress",JSON.stringify({ip:ip.value.trim(),port:port.value}));}catch(error){}
      window.location.assign(address);
    }catch(error){message.textContent=error.message;}
  });
  preview();
}
