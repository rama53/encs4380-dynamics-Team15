const $=id=>document.getElementById(id);
const TF=0.02, DT=0.001, R=1, UMAX=100, UMIN=-100;
let C={}, M={}, plant=null, spec=null, submitted=false, round=0, hist=[];

/* ---- sliders, same ranges and look as P1 ---- */
const SL={
  Kp:["Kp (proportional)",0,20,0.1,1,1,"Faster response, but leaves error on its own."],
  Ki:["Ki (integral)",0,20,0.1,0,1,"Removes steady-state error."],
  Kd:["Kd (derivative)",0,5,0.05,0,2,"Damps overshoot."]
};
$("ctlsl").innerHTML=Object.keys(SL).map(id=>{const d=SL[id];return `<div class="sl"><label for="${id}">${d[0]} <output id="${id}Out" for="${id}"></output></label><input id="${id}" type="range" min="${d[1]}" max="${d[2]}" step="${d[3]}" value="${d[4]}"><small>${d[6]}</small></div>`}).join("");
const val=id=>+$(id).value;
const css=n=>getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const fmt=(x,d)=>x===null||x===undefined?"—":x.toFixed(d);
const rnd=(a,b)=>a+Math.random()*(b-a), r2=x=>Math.round(x*100)/100;

function showVals(){Object.keys(SL).forEach(id=>{$(id+"Out").textContent=val(id).toFixed(SL[id][5])})}
Object.keys(SL).forEach(id=>$(id).addEventListener("input",showVals));
showVals();
