const fs=require('fs'),vm=require('vm');
const root=require('path').resolve(__dirname,'..')+'/';
const html=fs.readFileSync(root+'freeway/viewer_template.html.in','utf8'),data=JSON.parse(fs.readFileSync(root+'results/freeway/data.json','utf8'));
new vm.Script(html.split('<script>')[1].split('</script>')[0].replace('__SIMULATION_DATA__','{}'));
let polygons=[],path=[],stack=[];
const c=new Proxy({
 save(){stack.push({fillStyle:this.fillStyle});},restore(){Object.assign(this,stack.pop()||{});},
 beginPath(){path=[];},moveTo(x,y){path.push([x,y]);},lineTo(x,y){path.push([x,y]);},
 fill(){if(this.fillStyle==='#edaa65'&&path.length===4)polygons.push({p:[...path],ramp:true});},
 fillRect(x,y,w,h){if(['#edaa65','#46ba97','#7ebae6'].includes(this.fillStyle)&&Math.abs(w-5*1.125)<1e-6&&h>10)polygons.push({p:[[x,y],[x+w,y],[x+w,y+h],[x,y+h]],ramp:this.fillStyle==='#edaa65'});}
},{get:(o,k)=>k in o?o[k]:(()=>{})});
const box={cfg:data.config,Math,Map,Set,trackedId:null,mode:'adaptive',setup:()=>[c,1200,295]};vm.createContext(box);
vm.runInContext(html.slice(html.indexOf('function rampSignal('),html.indexOf('function axes(')),box);
function intersects(a,b){for(const p of [a,b])for(let i=0;i<p.length;i++){const q=p[(i+1)%p.length],v=p[i],axis=[q[1]-v[1],v[0]-q[0]];if(Math.hypot(...axis)<1e-8)continue;const pa=a.map(z=>z[0]*axis[0]+z[1]*axis[1]),pb=b.map(z=>z[0]*axis[0]+z[1]*axis[1]);if(Math.max(...pa)<=Math.min(...pb)+1e-6||Math.max(...pb)<=Math.min(...pa)+1e-6)return false;}return true;}
let counts={},examples=[];
for(const [mode,run] of Object.entries(data.runs)){box.mode=mode;let collisions=0;
for(let i=0;i<run.frames.length-1;i++)for(const f of [0,.25,.5,.75]){polygons=[];box.drawRoad(box.replayFrame(run,i+f));const near=polygons.filter(o=>o.p.some(p=>p[0]>40+180*1.125&&p[0]<40+330*1.125));for(let a=0;a<near.length;a++)for(let b=a+1;b<near.length;b++){if(!(near[a].ramp||near[b].ramp))continue;if(intersects(near[a].p,near[b].p)){collisions++;if(examples.length<3)examples.push({mode,time:i+f,a:near[a],b:near[b]});}}}
counts[mode]=collisions;}
console.log(JSON.stringify({counts,examples}));

if(Object.values(counts).some(n=>n>0))throw new Error('Ramp vehicle display overlap near merge');
