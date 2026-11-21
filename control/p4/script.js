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

/* ---- RK4 simulation of the closed loop (same model as P1). State s = [y, v, I, xd] ---- */
function rhs(s,P){
  const e=P.r-s[0], D=P.Kd*(e-s[3])/TF, uu=P.Kp*e+P.Ki*s[2]+D;
  const u=Math.min(P.umax,Math.max(P.umin,uu));
  let dI=e; if(P.aw && uu!==u && (uu-u)*e>0) dI=0;
  let dy,dv;
  if(P.type===1){dy=(P.K*u-s[0])/P.tau;dv=0}
  else{dy=s[1];dv=P.wn*P.wn*(P.K*u-s[0])-2*P.zeta*P.wn*s[1]}
  return [dy,dv,dI,(e-s[3])/TF,u];
}
function simulate(P){
  const n=Math.round(P.T/DT), stride=Math.max(1,Math.round(n/1500));
  let s=[0,0,0,0], t=[], y=[], u=[], bad=false;
  const add=(a,k,h)=>a.map((v,j)=>j<4?v+h*k[j]:0);
  for(let i=0;i<=n;i++){
    const k1=rhs(s,P);
    if(i%stride===0){t.push(i*DT);y.push(s[0]);u.push(k1[4])}
    const k2=rhs(add(s,k1,DT/2),P), k3=rhs(add(s,k2,DT/2),P), k4=rhs(add(s,k3,DT),P);
    for(let j=0;j<4;j++) s[j]+=DT/6*(k1[j]+2*k2[j]+2*k3[j]+k4[j]);
    if(!isFinite(s[0])||Math.abs(s[0])>1e9){bad=true;break}
  }
  return {t,y,u,bad};
}
/* same definitions as P1: 10–90% rise, overshoot, last exit from the ±2% band, error at the end */
function metrics(res,r){
  const t=res.t,y=res.y,n=y.length,m={rise:null,os:0,ts:null,sse:null};
  let t10=null,t90=null,ymax=-Infinity,lastOut=-1;
  for(let i=0;i<n;i++){
    if(t10===null&&y[i]>=0.1*r)t10=t[i];
    if(t90===null&&y[i]>=0.9*r)t90=t[i];
    if(y[i]>ymax)ymax=y[i];
    if(Math.abs(y[i]-r)>0.02*r)lastOut=i;
  }
  if(t10!==null&&t90!==null)m.rise=t90-t10;
  m.os=Math.max(0,(ymax-r)/r*100);
  if(lastOut<n-1)m.ts=lastOut<0?0:t[lastOut+1];
  m.sse=(r-y[n-1])/r*100;
  return m;
}

/* ---- pass or fail on each part of the spec ---- */
function evaluate(m,s,bad){
  const parts=[
    ["Overshoot","≤ "+s.os+" %",fmt(m.os,1)+" %",35,!bad&&m.os<=s.os],
    ["Settling time (2%)","≤ "+s.ts.toFixed(1)+" s",m.ts===null?"not settled":fmt(m.ts,2)+" s",35,!bad&&m.ts!==null&&m.ts<=s.ts],
    ["Steady-state error","|e| ≤ "+s.sse+" %",fmt(m.sse,2)+" %",30,!bad&&Math.abs(m.sse)<=s.sse]
  ];
  return {parts,score:parts.reduce((a,x)=>a+(x[4]?x[3]:0),0),passed:parts.filter(x=>x[4]).length};
}

/* ---- plot: P1's look plus the spec band and limit lines ---- */
function plot(cv,t,y,T,s){
  const r=window.devicePixelRatio||1,w=cv.clientWidth,h=cv.clientHeight;
  cv.width=Math.round(w*r);cv.height=Math.round(h*r);
  const c=cv.getContext("2d");c.setTransform(r,0,0,r,0,0);c.clearRect(0,0,w,h);
  const L=52,Rr=16,Tp=18,B=46,pw=w-L-Rr,ph=h-Tp-B;
  let lo=0,hi=R*(1+s.os/100);
  y.forEach(v=>{if(v<lo)lo=v;if(v>hi)hi=v});
  lo=Math.max(lo,-2*R);hi=Math.min(hi,3*R);
  const pad=(hi-lo)*0.08;hi+=pad;if(lo<0)lo-=pad;
  const X=v=>L+v/T*pw,Y=v=>Tp+(hi-v)/(hi-lo)*ph;
  c.font="12px system-ui,Arial,sans-serif";c.lineWidth=1;
  c.globalAlpha=0.15;c.fillStyle=C.m2;c.fillRect(L,Y(R*1.02),pw,Y(R*0.98)-Y(R*1.02));c.globalAlpha=1;
  for(let i=0;i<=5;i++){
    const tt=T*i/5;c.strokeStyle=C.line;c.beginPath();c.moveTo(X(tt),Tp);c.lineTo(X(tt),Tp+ph);c.stroke();
    c.fillStyle=C.muted;c.textAlign="center";c.fillText(+tt.toFixed(1),X(tt),Tp+ph+16);
  }
  for(let i=0;i<=4;i++){
    const v=lo+(hi-lo)*i/4;c.strokeStyle=C.line;c.beginPath();c.moveTo(L,Y(v));c.lineTo(L+pw,Y(v));c.stroke();
    c.fillStyle=C.muted;c.textAlign="right";c.fillText(v.toFixed(2),L-8,Y(v)+4);
  }
  c.fillStyle=C.ink;c.textAlign="center";c.fillText("Time (s)",L+pw/2,Tp+ph+36);
  c.save();c.translate(12,Tp+ph/2);c.rotate(-Math.PI/2);c.fillText("Output y(t)",0,0);c.restore();
  c.lineWidth=2;c.setLineDash([6,5]);
  [[R,C.muted],[R*(1+s.os/100),C.m1]].forEach(l=>{c.strokeStyle=l[1];c.beginPath();c.moveTo(L,Y(l[0]));c.lineTo(L+pw,Y(l[0]));c.stroke()});
  c.strokeStyle=C.m2;c.beginPath();c.moveTo(X(s.ts),Tp);c.lineTo(X(s.ts),Tp+ph);c.stroke();
  c.setLineDash([]);
  c.strokeStyle=C.curve;c.lineWidth=3;c.beginPath();
  y.forEach((v,i)=>{const px=X(t[i]),py=Y(Math.min(hi,Math.max(lo,v)));i?c.lineTo(px,py):c.moveTo(px,py)});
  c.stroke();
}

/* ---- main redraw ---- */
function draw(){
  Object.keys(SL).forEach(id=>{$(id+"Out").textContent=val(id).toFixed(SL[id][5])});
  C={line:css("--line"),muted:css("--muted"),ink:css("--ink"),curve:css("--curve"),m1:css("--mark"),m2:css("--mark2")};
  const P=Object.assign({},plant,{Kp:val("Kp"),Ki:val("Ki"),Kd:val("Kd"),r:R,umax:UMAX,umin:UMIN,T:spec.T,aw:true});
  const res=simulate(P); M=metrics(res,R);
  plot($("c1"),res.t,res.y,spec.T,spec);
  const peak=Math.max(...res.u.map(Math.abs)), sat=res.u.some(v=>v>=UMAX-1e-9||v<=UMIN+1e-9);
  $("mnote").textContent="Peak |u|: "+peak.toFixed(1)+(sat?". It hits the ±100 limit, so raising the gains further will not make the loop much faster.":".");
  $("warn").textContent=res.bad?"Unstable: at these gains the output grows without bound.":"";
  const ev=evaluate(M,spec,res.bad);
  $("sbody").innerHTML=ev.parts.map(x=>`<tr><td>${x[0]}</td><td>${x[1]}</td><td>${x[2]}</td><td>${x[3]}</td><td><span class="pill ${x[4]?"pass":"fail"}">${x[4]?"Pass":"Fail"}</span></td></tr>`).join("");
  $("now").textContent=ev.score;
  return ev;
}

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
  showChallenge();draw();
}
$("newPlant").addEventListener("click",()=>start(false));
$("practice").addEventListener("click",()=>start(true));
$("submit").addEventListener("click",()=>{
  if(submitted)return;
  const ev=draw();submitted=true;$("submit").disabled=true;
  $("reveal").textContent=(ev.passed===3?"All three parts pass. ":ev.passed+" of 3 parts pass. ")+"The plant was "+plantText(plant)+". Press \"New hidden plant\" for the next one.";
  if(plant.practice)return;
  round++;hist.push({n:round,txt:plantText(plant),g:[val("Kp"),val("Ki"),val("Kd")],passed:ev.passed,score:ev.score});
  $("hist").innerHTML=hist.map(h=>`<tr><td>${h.n}</td><td>${h.txt}</td><td>${h.g[0].toFixed(1)}, ${h.g[1].toFixed(1)}, ${h.g[2].toFixed(2)}</td><td>${h.passed} / 3</td><td>${h.score}</td></tr>`).join("");
  $("avg").textContent=Math.round(hist.reduce((a,h)=>a+h.score,0)/hist.length)+" / 100 over "+hist.length+" plant"+(hist.length>1?"s":"");
});

/* ---- Predict, then check ---- */
const QS=[
 {q:"The output settles below the setpoint, so the steady-state error part fails. Which gain should you raise?",o:["Kp","Ki","Kd"],a:1,
  e:"The integral keeps adding up the error until it is zero. Kp only makes the error smaller, and Kd does nothing once the output is steady."},
 {q:"The overshoot is 25% but the spec allows 10%. Which change usually brings it down without making the response much slower?",o:["Raise Kd","Raise Ki","Raise Kp"],a:0,
  e:"Derivative reacts to how fast the error is changing, so it brakes the output as it approaches the setpoint. Raising Ki or Kp usually adds overshoot."},
 {q:"You keep raising Kp to settle faster, and u starts hitting the ±100 limit. What happens to the settling time?",o:["It keeps getting shorter","It stops improving, because the actuator cannot push harder","It becomes zero"],a:1,
  e:"Once u is saturated, the plant gets the same maximum input whatever the gains say, so the response cannot get faster. Watch the peak |u| note under the plot."}
];
let qi=0;
function predict(){
  const p=QS[qi];
  $("pQn").textContent="Question "+(qi+1)+" of "+QS.length;$("pQ").textContent=p.q;$("pResult").textContent="";
  $("pOpts").innerHTML=p.o.map((o,i)=>`<label><input type="radio" name="pred" value="${i}"> ${o}</label>`).join("");
  $("pGo").disabled=false;$("pNext").hidden=true;
}
$("pGo").addEventListener("click",()=>{
  const p=QS[qi],sel=document.querySelector("input[name=pred]:checked");
  if(!sel){$("pResult").textContent="Pick an answer first.";return}
  $("pResult").textContent=(+sel.value===p.a?"Correct. ":"Not quite. ")+p.e+" Now try it with the sliders.";
  document.querySelectorAll("input[name=pred]").forEach(r=>r.disabled=true);
  $("pGo").disabled=true;$("pNext").hidden=false;
});
$("pNext").addEventListener("click",()=>{qi=(qi+1)%QS.length;predict()});

/* ---- wiring ---- */
Object.keys(SL).forEach(id=>$(id).addEventListener("input",draw));
window.addEventListener("resize",draw);
matchMedia("(prefers-color-scheme: dark)").addEventListener("change",draw);
predict();start(false);
