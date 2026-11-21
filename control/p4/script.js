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

/* ---- hidden plant, same ranges as Module 4's random sensors ---- */
function newPlant(practice){
  let p;
  if(practice) p={type:1,K:1,tau:1,practice:true};
  else if(Math.random()<0.5) p={type:1,K:r2(rnd(0.5,4)),tau:r2(rnd(0.5,6))};
  else p={type:2,K:1,zeta:r2(rnd(0.1,0.9)),wn:r2(rnd(1,8))};
  /* settling-time limit: 40% of the plant's own open-loop settling time (4τ or 4/(ζωn)).
     Checked offline: every plant in these ranges can pass with gains inside the slider ranges. */
  const olts=p.type===1?4*p.tau:4/(p.zeta*p.wn);
  const ts=Math.max(0.5,Math.round(olts*0.4*10)/10);
  return {p,s:{os:10,ts,sse:1,T:Math.max(3,Math.ceil(ts*3))}};
}
function plantText(p){
  return p.type===1?`first order, K = ${p.K}, τ = ${p.tau} s`:`second order, K = 1, ζ = ${p.zeta}, ωn = ${p.wn} rad/s`;
}

function showVals(){Object.keys(SL).forEach(id=>{$(id+"Out").textContent=val(id).toFixed(SL[id][5])})}

function showChallenge(){
  const p=plant;
  $("ptag").textContent=p.practice?"Practice":"Plant #"+(round+1);
  $("pdesc").textContent=p.practice
    ?"Practice plant: first order, K = 1, τ = 1 s (shown so you can check your work). It does not count in your score."
    :"Hidden "+(p.type===1?"first-order":"second-order")+" plant. Its numbers are revealed after you submit.";
  $("spec").innerHTML=[["Overshoot","≤ "+spec.os+" %"],["Settling time (2%)","≤ "+spec.ts.toFixed(1)+" s"],["Steady-state error","≤ "+spec.sse+" %"]]
    .map(x=>`<div><span>${x[0]}</span><b>${x[1]}</b></div>`).join("");
  $("reveal").textContent="";
  $("submit").disabled=false;submitted=false;
}
function start(practice){
  const n=newPlant(practice);plant=n.p;spec=n.s;
  $("Kp").value=1;$("Ki").value=0;$("Kd").value=0;
  showChallenge();showVals();
}
$("newPlant").addEventListener("click",()=>start(false));
$("practice").addEventListener("click",()=>start(true));
Object.keys(SL).forEach(id=>$(id).addEventListener("input",showVals));
start(false);
