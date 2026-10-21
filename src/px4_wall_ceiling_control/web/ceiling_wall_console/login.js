"use strict";
const loginForm=document.getElementById("loginForm"),loginButton=document.getElementById("loginButton"),loginMessage=document.getElementById("loginMessage");
loginForm.addEventListener("submit",async event=>{
  event.preventDefault();if(loginButton.disabled||!loginForm.reportValidity())return;
  loginButton.disabled=true;loginMessage.textContent="正在连接…";
  try{
    const data=Object.fromEntries(new FormData(loginForm));
    const response=await fetch("/api/login",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(data),signal:AbortSignal.timeout(10000)});
    const result=await response.json();if(!response.ok)throw new Error(result.error||"连接失败");
    document.getElementById("accessCode").value="";window.location.replace("/");
  }catch(error){loginMessage.textContent=error.name==="TimeoutError"?"连接超时，请确认设备仍在同一网络":error.message;loginMessage.classList.add("error");}
  finally{loginButton.disabled=false;}
});

fetch("/api/info",{cache:"no-store"}).then(response=>response.json()).then(info=>{
  if(info.offline){const hint=document.getElementById("offlineLoginHint");hint.hidden=false;hint.textContent="离线模拟，不连接设备。访问码：offline";document.getElementById("accessCode").value="offline";}
}).catch(()=>{});
