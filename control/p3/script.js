"use strict";
/*SIM*/
function sim(P,Ts){
  const N=Math.ceil(P.tEnd/Ts-1e-9);
  const f=(x,u)=>P.type===1?[(P.K*u-x[0])/P.tau,0]
    :[x[1],P.K*P.wn*P.wn*u-2*P.zeta*P.wn*x[1]-P.wn*P.wn*x[0]];
  let x=[0,0],I=0,prev=0,unstable=false;
  const T=[0],Y=[0],UT=[],UV=[];
  for(let k=0;k<N&&!unstable;k++){
    const start=k*Ts,end=Math.min((k+1)*Ts,P.tEnd);
    const n=Math.max(1,Math.ceil((end-start)/0.001-1e-9)),h=(end-start)/n;
    const e=P.sp-x[0];
    const iNew=I+P.Ki*Ts*e;
    const u=P.Kp*e+iNew+P.Kd*(e-prev)/Ts;
    const us=Math.max(-P.U,Math.min(P.U,u));
    // clamping anti-windup
    if(!P.aw||!((u>P.U&&e>0)||(u<-P.U&&e<0))) I=iNew;
    prev=e;
    UT.push(start,end);UV.push(us,us);
    for(let j=0;j<n;j++){
      const k1=f(x,us),k2=f([x[0]+h/2*k1[0],x[1]+h/2*k1[1]],us),
            k3=f([x[0]+h/2*k2[0],x[1]+h/2*k2[1]],us),k4=f([x[0]+h*k3[0],x[1]+h*k3[1]],us);
      x=[x[0]+h/6*(k1[0]+2*k2[0]+2*k3[0]+k4[0]),x[1]+h/6*(k1[1]+2*k2[1]+2*k3[1]+k4[1])];
      T.push(j===n-1?end:start+(j+1)*h);Y.push(x[0]);
      if(!isFinite(x[0])||Math.abs(x[0])>1e3*Math.max(1,P.sp)){unstable=true;break}
    }
  }
  return {T,Y,UT,UV,unstable,m:unstable?{unstable:true}:metrics(T,Y,P.sp)};
}
function metrics(T,Y,sp){
  const crossing=(i,level)=>T[i-1]+(T[i]-T[i-1])*(level-Y[i-1])/(Y[i]-Y[i-1]);
  let t10=null,t90=null,peak=0,lastOutside=-1;
  for(let i=0;i<Y.length;i++){
    if(t10===null&&Y[i]>=.1*sp)t10=crossing(i,.1*sp);
    if(t90===null&&Y[i]>=.9*sp)t90=crossing(i,.9*sp);
    peak=Math.max(peak,Y[i]);
    if(Y[i]<.98*sp||Y[i]>1.02*sp)lastOutside=i;
  }
  const start=T.at(-1)*.95;
  let area=0;
  for(let i=T.length-1;i>0&&T[i]>start;i--){
    const left=Math.max(T[i-1],start),width=T[i]-left;
    const yLeft=Y[i]-(Y[i]-Y[i-1])*width/(T[i]-T[i-1]);
    area+=width*(Y[i]+yLeft)/2;
  }
  const settle=lastOutside===T.length-1?null:lastOutside<0?0:
    crossing(lastOutside+1,Y[lastOutside]<sp?.98*sp:1.02*sp);
  return {rise:t90===null?null:t90-t10,os:Math.max(0,(peak-sp)/sp*100),
    settle,sse:(sp-area/(T.at(-1)-start))/sp*100};
}
/*END*/

const $=id=>document.getElementById(id),css=n=>getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const DEF={type:1,K:1,tau:1,wn:3,zeta:.5,Kp:9,Ki:0,Kd:0,lts:-2,U:10,sp:1,tEnd:10,aw:true};
let S={...DEF};
const periodUs=lts=>Math.round(10**lts*1e6);
const tsOf=s=>periodUs(s.lts)/1e6;
const msTxt=v=>periodUs(v)/1000+' ms';
const SL={
  p1sl:[['K','Plant gain K',.1,5,.1,v=>v.toFixed(1)],['tau','Time constant τ (s)',.1,5,.1,v=>v.toFixed(1)]],
  p2sl:[['K','Plant gain K',.1,5,.1,v=>v.toFixed(1)],['wn','ωn (rad/s)',.5,10,.1,v=>v.toFixed(1)],['zeta','ζ',0,2,.05,v=>v.toFixed(2)]],
  ctlsl:[['Kp','Kp',0,20,.1,v=>v.toFixed(1)],['Ki','Ki',0,20,.1,v=>v.toFixed(1)],['Kd','Kd',0,5,.01,v=>v.toFixed(2)],
    ['lts','Sampling time Ts',-3,0,'any',msTxt],['U','Control limit ±U',1,50,1,v=>'±'+v],['sp','Setpoint r',.1,5,.1,v=>v.toFixed(1)],['tEnd','Run length (s)',2,30,1,v=>v+' s']]
};
const FMT={};
Object.entries(SL).forEach(([box,defs])=>defs.forEach(([k,lab,mn,mx,st,fm])=>{
  FMT[k]=fm;
  const d=document.createElement('div');d.className='sl';
  d.innerHTML=`<label for="s_${box}_${k}">${lab}<output id="o_${box}_${k}" data-k="${k}"></output></label><input type="range" id="s_${box}_${k}" data-k="${k}" min="${mn}" max="${mx}" step="${st}">`;
  $(box).appendChild(d);
}));
document.querySelectorAll('input[type=range]').forEach(el=>el.addEventListener('input',()=>{const key=el.dataset.k;
  S[key]=key==='lts'?Math.round(+el.value*100)/100:+el.value;
  scheduleUpdate();}));
$('aw').onchange=()=>{S.aw=$('aw').checked;update()};
function setType(t){S.type=t;$('b1').setAttribute('aria-pressed',t===1);$('b2').setAttribute('aria-pressed',t===2);$('p1sl').hidden=t!==1;$('p2sl').hidden=t!==2;update()}
$('b1').onclick=()=>setType(1);$('b2').onclick=()=>setType(2);
$('reset').onclick=()=>{S={...DEF};setType(1)};
document.querySelectorAll('[data-gains]').forEach(b=>b.onclick=()=>{
  const [p,i,d]=b.dataset.gains.split(',').map(Number);Object.assign(S,{type:1,K:1,tau:1,Kp:p,Ki:i,Kd:d,U:50,sp:1,lts:-3});setType(1)});
document.querySelectorAll('[data-ts]').forEach(b=>b.onclick=()=>{S.lts=+b.dataset.ts;update()});

/* ---- plotting ---- */
function plot(cv,series,o){
  const dpr=window.devicePixelRatio||1,w=cv.clientWidth,h=cv.clientHeight;
  cv.width=w*dpr;cv.height=h*dpr;const g=cv.getContext('2d');g.scale(dpr,dpr);
  const L=48,R=12,Tp=12,B=34,pw=w-L-R,ph=h-Tp-B;
  const xs=series.flatMap(s=>s.x),ys=series.flatMap(s=>s.y).filter(v=>v!==null).concat(o.hl||[]);
  const x0=o.xlog?Math.log10(Math.min(...xs)):Math.min(...xs),x1=o.xlog?Math.log10(Math.max(...xs)):Math.max(...xs);
  const y0=o.ymin??Math.min(...ys);let y1=o.ymax??Math.max(...ys);if(y1-y0<1e-9)y1=y0+1;
  const px=v=>L+((o.xlog?Math.log10(v):v)-x0)/(x1-x0)*pw,py=v=>Tp+ph-(v-y0)/(y1-y0)*ph;
  g.font='12px system-ui,sans-serif';g.fillStyle=css('--muted');g.strokeStyle=css('--line');g.lineWidth=1;
  for(let i=0;i<=4;i++){const v=y0+(y1-y0)*i/4,y=py(v);g.beginPath();g.moveTo(L,y);g.lineTo(L+pw,y);g.stroke();g.textAlign='right';g.fillText(+v.toPrecision(3),L-6,y+4)}
  g.textAlign='center';
  (o.xlog?[.001,.01,.1,1]:Array.from({length:5},(_,i)=>x0+(x1-x0)*i/4)).forEach(v=>g.fillText(+v.toPrecision(2),px(v),h-16));
  g.fillText(o.xl,L+pw/2,h-3);g.save();g.translate(12,Tp+ph/2);g.rotate(-Math.PI/2);g.fillText(o.yl,0,0);g.restore();
  (o.hl||[]).forEach(v=>{g.strokeStyle=css('--mark2');g.setLineDash([6,4]);g.lineWidth=1.5;g.beginPath();g.moveTo(L,py(v));g.lineTo(L+pw,py(v));g.stroke();g.setLineDash([])});
  g.save();g.beginPath();g.rect(L,Tp,pw,ph);g.clip();
  series.forEach(s=>{g.strokeStyle=css(s.c);g.lineWidth=s.w||2.5;g.setLineDash(s.dash||[]);g.beginPath();
    let connected=false;
    s.x.forEach((xv,i)=>{
      const value=s.y[i];
      if(value===null){connected=false;return}
      const X=px(xv),Y=py(value);
      if(connected)g.lineTo(X,Y);else g.moveTo(X,Y);
      connected=true;
    });
    g.stroke();
    if(s.points){
      s.x.forEach((xv,i)=>{
        const X=px(xv),value=s.y[i],Y=py(value??y1);
        g.beginPath();
        if(value===null){
          g.moveTo(X-4,Y);g.lineTo(X+4,Y+8);
          g.moveTo(X+4,Y);g.lineTo(X-4,Y+8);
          g.stroke();
        }else{
          g.arc(X,Y,2.5,0,2*Math.PI);g.fillStyle=css(s.c);g.fill();
        }
      });
    }});
  g.restore();g.setLineDash([]);
  if(o.mark){g.strokeStyle=css('--ink');g.lineWidth=1.5;g.setLineDash([3,4]);g.beginPath();g.moveTo(px(o.mark),Tp);g.lineTo(px(o.mark),Tp+ph);g.stroke();g.setLineDash([])}
}
let sweepKey='',sweepResults=[];
function sweep(P){
  const {lts,...settings}=P;
  const key=JSON.stringify(settings);
  if(key!==sweepKey){
    sweepResults=Array.from({length:25},(_,i)=>{
      const Ts=periodUs(-3+3*i/24)/1e6,r=sim(P,Ts);
      return {Ts,os:r.m.os??null,st:r.m.settle??null};
    });
    sweepKey=key;
  }
  return sweepResults;
}

/* ---- Arduino code ---- */
function arduino(P,Ts){
  const g=v=>(+v).toFixed(3);
  const wind=P.aw
  ?`  // anti-windup: keep the new integral unless the output is
  // saturated and the error pushes it further out
  bool pushingOut = (u > U_MAX && e > 0) || (u < U_MIN && e < 0);
  if (!pushingOut) integral = iNew;`
  :`  integral = iNew;   // no anti-windup`;
  return `// Digital PID, Ts = ${g(Ts*1000)} ms
// u[k] = Kp*e[k] + Ki*Ts*sum(e) + Kd*(e[k]-e[k-1])/Ts
const float Kp = ${g(P.Kp)};
const float Ki = ${g(P.Ki)};
const float Kd = ${g(P.Kd)};
const unsigned long TS_US = ${Math.round(Ts*1e6)}UL;
const float Ts = TS_US / 1000000.0f;             // seconds
const float U_MIN = ${-P.U}.0, U_MAX = ${P.U}.0;   // control limits
float setpoint = ${g(P.sp)};

float integral = 0, prevErr = 0;
unsigned long lastUs;

float readSensor() { return 0; }    // TODO: encoder or sensor reading
void writeActuator(float u) { }     // TODO: map u to PWM

void setup() {
  lastUs = micros() - TS_US;         // take the first sample immediately
}

void loop() {
  unsigned long now = micros();
  if (now - lastUs < TS_US) return;  // wait for the next sample
  lastUs += TS_US;                   // preserve the regular sample schedule
  if (now - lastUs >= TS_US) lastUs = now; // skip missed ticks after an overrun

  float e = setpoint - readSensor();
  float iNew = integral + Ki * Ts * e;
  float u = Kp * e + iNew + Kd * (e - prevErr) / Ts;
${wind}
  prevErr = e;

  writeActuator(constrain(u, U_MIN, U_MAX));
}`;
}

/* ---- update ---- */
const fmt=(v,d=2,u='')=>v===null||v===undefined?'—':v.toFixed(d)+u;
let updateFrame;
function scheduleUpdate(){
  cancelAnimationFrame(updateFrame);
  updateFrame=requestAnimationFrame(update);
}
function update(){
  const Ts=tsOf(S),P={...S};
  document.querySelectorAll('input[type=range]').forEach(el=>{el.value=S[el.dataset.k]});
  document.querySelectorAll('output[data-k]').forEach(el=>{el.textContent=FMT[el.dataset.k](S[el.dataset.k])});
  $('aw').checked=S.aw;
  $('diagramPlant').textContent=S.type===1?'K/(τ·s + 1)':'Second order';
  $('plantEquation').textContent=S.type===1?'ẏ = (K·u − y)/τ':'ÿ = Kωn²·u − 2ζωn·ẏ − ωn²·y';
  const r=sim(P,Ts),m=r.m;
  $('cards').innerHTML=m.unstable
    ?`<div class="a"><span>Status</span><b>Unstable</b><small>The simulation exceeded its numerical guard</small></div>`
    :`<div class="a"><span>Rise time</span><b>${fmt(m.rise,3,' s')}</b><small>10% to 90% of r</small></div>
      <div class="b"><span>Overshoot</span><b>${fmt(m.os,1,' %')}</b><small>Peak above r</small></div>
      <div class="c"><span>Settling time</span><b>${m.settle===null?'not settled':fmt(m.settle,2,' s')}</b><small>Stays within ±2% of r</small></div>
      <div class="a"><span>End-of-run error</span><b>${fmt(m.sse,2,' %')}</b><small>Average over the last 5% of the run</small></div>`;
  let limitedTime=0;
  for(let i=0;i<r.UV.length;i+=2){
    if(Math.abs(r.UV[i])>=S.U)limitedTime+=Math.min(r.T.at(-1),r.UT[i+1])-r.UT[i];
  }
  const atLim=100*limitedTime/r.T.at(-1);
  const pole=S.type===1&&S.Ki===0&&S.Kd===0?sampledPole(S,Ts):null;
  $('warn').textContent=r.unstable?'Simulation stopped after exceeding its numerical guard.':
    pole!==null&&Math.abs(pole)>=1
      ?`Unsaturated P-only pole z = ${pole.toFixed(2)}. The linear loop is at or beyond the stability boundary. The actuator limit bounds the plotted output.`
      :atLim>50?`The control signal is at its limit for ${atLim.toFixed(0)}% of the run.`:'';
  $('stabilityNote').textContent=`Ts = ${msTxt(S.lts)} · ${r.UT.length/2} controller updates · ${r.T.at(-1).toFixed(2)} s simulated`;
  $('compute').textContent='Simulated with RK4 in steps of at most 1 ms.';
  plot($('c1'),[{x:[0,S.tEnd],y:[S.sp,S.sp],c:'--muted',w:1.8,dash:[6,4]},{x:r.T,y:r.Y,c:'--curve'}],
    {xl:'time (s)',yl:'output',ymin:Math.min(0,...r.Y),ymax:Math.max(S.sp*1.2,Math.max(...r.Y))});
  plot($('c2'),[{x:r.UT,y:r.UV,c:'--mark'}],{xl:'time (s)',yl:'control u',hl:[S.U,-S.U],ymin:-S.U*1.1,ymax:S.U*1.1});
  const sw=sweep(P);
  plot($('c3'),[{x:sw.map(s=>s.Ts),y:sw.map(s=>s.os),c:'--mark',points:true}],{xl:'sampling time Ts (s, log)',yl:'overshoot (%)',xlog:true,ymin:0,mark:Ts});
  plot($('c4'),[{x:sw.map(s=>s.Ts),y:sw.map(s=>s.st),c:'--curve',points:true}],{xl:'sampling time Ts (s, log)',yl:'settling time (s)',xlog:true,ymin:0,ymax:P.tEnd,mark:Ts});
  const code=arduino(P,Ts);
  if($('code').textContent!==code){
    $('code').textContent=code;
    $('copied').textContent='';
  }
}
$('copy').onclick=async()=>{
  const t=$('code').textContent;
  try{
    await navigator.clipboard.writeText(t);
    $('copied').textContent='Copied';
  }catch{
    const selection=window.getSelection(),range=document.createRange();
    range.selectNodeContents($('code'));
    $('code').focus();
    selection.removeAllRanges();
    selection.addRange(range);
    $('copied').textContent='Code selected. Press Ctrl+C or Command+C to copy.';
  }
};

/* ---- predict, then check ---- */
const BASE={type:1,K:1,tau:1,Kp:9,Ki:0,Kd:0,U:10,sp:1,tEnd:10,aw:true};
const L5=Math.log10(.5),L10=Math.log10(.01);
const Q=[
 {q:'With P only and Kp = 9, what does Ts = 500 ms do to the loop?',setup:'Experiment: K = 1, τ = 1 s, P only, Kp = 9, limit ±10, Ts = 500 ms.',
  set:{...BASE,lts:L5},opts:['The response is smooth but slower','The output swings back and forth and keeps growing until the control limit stops it','Nothing changes compared with Ts = 10 ms'],ans:1,
  why:'With Ts = 500 ms the sampled loop has a pole at about −2.9, outside the unit circle. The limit stops the growth, so it ends in a steady swing.',
  cmp:[{l:'Ts = 10 ms',o:{lts:L10}},{l:'Ts = 500 ms',o:{lts:L5}}]},
 {q:'The control limit is low, so the actuator saturates at the start. What does anti-windup do?',setup:'Experiment: K = 1, τ = 1 s, Kp = 1, Ki = 3, limit ±2, r = 1.5, Ts = 10 ms. Anti-windup starts off.',
  set:{...BASE,Kp:1,Ki:3,U:2,sp:1.5,lts:L10,aw:false},opts:['Overshoot gets smaller','Overshoot stays the same','Overshoot gets larger'],ans:0,
  why:'While u is stuck at the limit, the integral keeps growing if anti-windup is off. That stored value must be unwound by overshooting. Clamping stops it from growing.',
  cmp:[{l:'Anti-windup off',o:{aw:false}},{l:'Anti-windup on',o:{aw:true}}]},
 {q:'A PI controller is stable at both Ts = 10 ms and Ts = 500 ms. Is the steady-state error still zero at 500 ms?',setup:'Experiment: K = 1, τ = 1 s, PI with Kp = 1, Ki = 1, limit ±10, 20 s run.',
  set:{...BASE,Kp:1,Ki:1,tEnd:20,lts:L5},opts:['Yes, the integral still removes it','No, a larger Ts leaves a permanent error'],ans:0,
  why:'The integral keeps accumulating while any error remains, so a stable loop settles only at zero error. Ts changes how fast and how smoothly it gets there.',
  cmp:[{l:'Ts = 10 ms',o:{lts:L10}},{l:'Ts = 500 ms',o:{lts:L5}}]}
];
let qi=0;
function showQ(){
  const q=Q[qi];
  $('pQn').textContent=`Question ${qi+1} of ${Q.length}`;$('pSetup').textContent=q.setup;$('pQ').textContent=q.q;
  $('pOpts').innerHTML=q.opts.map((t,i)=>`<label><input type="radio" name="pq" value="${i}"> ${t}</label>`).join('');
  $('pResult').textContent='';$('pLive').hidden=true;$('pNext').hidden=true;$('pGo').disabled=false;
}
$('pGo').onclick=()=>{
  const sel=document.querySelector('input[name=pq]:checked'),q=Q[qi];
  if(!sel){$('pResult').textContent='Choose an answer first.';return}
  S={...DEF,...q.set};setType(S.type);
  $('pResult').textContent=(+sel.value===q.ans?'Correct. ':'Not quite. ')+q.why;
  $('pLive').innerHTML=q.cmp.map(c=>{const p={...S,...c.o},r=sim(p,tsOf(p));
    return `<div><b>${c.l}:</b> ${r.unstable?'unstable':`overshoot ${r.m.os.toFixed(1)} %, end-of-run error ${r.m.sse.toFixed(2)} %`}</div>`}).join('');
  $('pLive').hidden=false;$('pGo').disabled=true;$('pNext').hidden=qi>=Q.length-1;
};
$('pNext').onclick=()=>{qi++;showQ()};

/* ---- validation ---- */
function sampledPole(P,Ts){
  const a=Math.exp(-Ts/P.tau);
  return a-P.K*P.Kp*(1-a);
}
function runChecks(){
  const B={...BASE,U:100,tEnd:3},rows=[];
  const c1=sim({...B,Kp:9},.001),tc=c1.T.find((t,i)=>c1.Y[i]>=(1-Math.exp(-1))*.9)??null;
  rows.push(['P only, Kp = 9: closed-loop time constant','0.100 s (±8%)',tc===null?'—':tc.toFixed(3)+' s',tc!==null&&Math.abs(tc-.1)<.008]);
  const e1=c1.m.sse;rows.push(['P only, Kp = 9: steady-state error','10 % (±0.5)',e1.toFixed(2)+' %',Math.abs(e1-10)<.5]);
  const c2=sim({...B,Kp:1,Ki:1,tEnd:20},.001);
  const pi={...B,Kp:1,Ki:1};
  const wn=Math.sqrt(pi.K*pi.Ki/pi.tau),zeta=(1+pi.K*pi.Kp)/(2*pi.tau*wn);
  rows.push(['PI, Kp = 1, Ki = 1: continuous reference poles',
    'ωn = 1 rad/s, ζ = 1',`ωn = ${wn.toFixed(2)} rad/s, ζ = ${zeta.toFixed(2)}`,
    Math.abs(wn-1)<.001&&Math.abs(zeta-1)<.001]);
  rows.push(['PI controller: zero steady-state error','0 % (within 0.1)',c2.m.sse.toFixed(3)+' %',Math.abs(c2.m.sse)<.1]);
  const a=sim({...BASE,Kp:1,Ki:3,U:2,sp:1.5,aw:true},.01).m.os,b=sim({...BASE,Kp:1,Ki:3,U:2,sp:1.5,aw:false},.01).m.os;
  rows.push(['Anti-windup reduces overshoot (limit ±2, r = 1.5)','On below off',`on ${a.toFixed(1)} %, off ${b.toFixed(1)} %`,a<b]);
  const s1=sim({...BASE,Kp:9},.1),s2=sim({...BASE,Kp:9},.5);
  const z1=sampledPole(BASE,.1),z2=sampledPole(BASE,.5);
  rows.push(['P only, Kp = 9: effect of sampling',
    '|z| < 1 at 100 ms, |z| > 1 at 500 ms',
    `z = ${z1.toFixed(2)} and ${z2.toFixed(2)}; overshoot ${s1.m.os.toFixed(1)}% and ${s2.m.os.toFixed(1)}%`,
    Math.abs(z1)<1&&Math.abs(z2)>1&&s2.m.os>s1.m.os]);
  $('vbody').innerHTML=rows.map(r=>`<tr><td>${r[0]}</td><td>${r[1]}</td><td>${r[2]}</td><td><span class="pill ${r[3]?'pass':'fail'}">${r[3]?'Pass':'Fail'}</span></td></tr>`).join('');
  const ok=rows.filter(r=>r[3]).length;$('validationStatus').textContent=`${ok} of ${rows.length} checks pass.`;
}
$('runChecks').onclick=runChecks;
addEventListener('resize',scheduleUpdate);
matchMedia('(prefers-color-scheme:dark)').addEventListener('change',scheduleUpdate);
showQ();update();runChecks();
