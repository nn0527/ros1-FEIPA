"""Build a snapshot fragment and a separate live local chart document."""
import json
import pathlib
import urllib.request

VIZ = pathlib.Path(r'C:\Users\60497\.codex\visualizations\2026\10\01\01a0f561-3e63-7092-8155-a10dfae5007e')
ROOT = pathlib.Path(__file__).resolve().parent
with urllib.request.urlopen('http://127.0.0.1:8765/data', timeout=5) as response:
    data = json.load(response)
(ROOT / 'telemetry_snapshot.json').write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')
template = (VIZ / 'px4-telemetry.template.html').read_text(encoding='utf-8')
fragment = template.replace('__PX4_DATA__', json.dumps(data, ensure_ascii=False).replace('</', '<\\/'))
(VIZ / 'px4-telemetry.html').write_text(fragment, encoding='utf-8')
styles = pathlib.Path(r'C:\Users\60497\.codex\plugins\cache\openai-bundled\visualize\1.0.45\skills\visualize\assets\visualize.css').read_text(encoding='utf-8')
live = '''<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>PX4 实时遥测</title><style>'''+styles+'''
body{margin:0;background:var(--background);color:var(--foreground);font-family:system-ui,"Microsoft YaHei",sans-serif;padding:24px}main{max-width:1100px;margin:auto}
</style></head><body><main>'''+fragment+'''</main>
<script>
(async function poll(){
 try {const response=await fetch('/data',{cache:'no-store'});if(!response.ok)throw Error('HTTP '+response.status);const data=await response.json();data.live=true;window.px4TelemetryUpdate(data);}
 catch(e){document.querySelector('[data-status]').textContent='遥测服务连接中断';}
 setTimeout(poll,250);
})();
(async function latest(){
 try {const started=performance.now();const response=await fetch('/latest',{cache:'no-store'});if(!response.ok)throw Error('HTTP '+response.status);const data=await response.json();window.px4TelemetryLatest(data);window.px4LatencySamples=window.px4LatencySamples||[];window.px4LatencySamples.push({http_ms:performance.now()-started,com4_age_ms:data.status.com4?.source_stamp_ms?Date.now()-data.status.com4.source_stamp_ms:null,com5_age_ms:data.status.com5?.source_stamp_ms?Date.now()-data.status.com5.source_stamp_ms:null});if(window.px4LatencySamples.length>200)window.px4LatencySamples.shift();const metric=document.querySelector('[data-network]');const last=window.px4LatencySamples.at(-1);metric.textContent=`页面请求 ${last.http_ms.toFixed(1)} ms · 当前数据年龄 COM4 ${last.com4_age_ms?.toFixed(0)??'—'} ms / COM5 ${last.com5_age_ms?.toFixed(0)??'—'} ms`;metric.dataset.samples=JSON.stringify(window.px4LatencySamples);}
 catch(e){document.querySelector('[data-status]').textContent='遥测服务连接中断';}
 setTimeout(latest,50);
})();
</script></body></html>'''
live=live.replace('https://cdn.jsdelivr.net/npm/d3@7.9.0/dist/d3.min.js','/d3.min.js')
(ROOT / 'telemetry_live.html').write_text(live, encoding='utf-8')
print('Charts generated;',len(data['samples']),'samples;',data['time'])
