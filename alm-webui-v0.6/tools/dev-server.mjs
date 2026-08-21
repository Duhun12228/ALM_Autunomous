/**
 * 개발용 정적 서버 — 캐시를 끈다.
 *
 * `python3 -m http.server` 는 Cache-Control 을 안 붙인다. 그러면 브라우저가
 * 휴리스틱 캐시를 적용해서, 번들을 다시 빌드해도 **새로고침이 옛 코드를 그대로
 * 쓰는** 일이 생긴다. 고쳤는데 안 고쳐진 것처럼 보이는 가장 흔한 원인이고,
 * 실제로 이 프로젝트에서 진단을 몇 번 헛돌게 만들었다.
 *
 *   node tools/dev-server.mjs [port]
 *   npm run serve
 */
import { createServer } from 'node:http';
import { createReadStream } from 'node:fs';
import { stat } from 'node:fs/promises';
import { extname, join, normalize, resolve } from 'node:path';

const ROOT = resolve(new URL('..', import.meta.url).pathname);
const PORT = Number(process.argv[2] || process.env.PORT || 8080);

const TYPES = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.mjs': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.map': 'application/json; charset=utf-8',
  '.svg': 'image/svg+xml',
  '.png': 'image/png',
  '.woff2': 'font/woff2',
};

createServer(async (req, res) => {
  const urlPath = decodeURIComponent((req.url || '/').split('?')[0]);
  // 루트 밖으로 나가는 경로는 거부한다 (개발 서버라도 파일 유출은 막는다)
  const target = resolve(join(ROOT, normalize(urlPath).replace(/^(\.\.[/\\])+/, '')));
  if (!target.startsWith(ROOT)) {
    res.writeHead(403).end('forbidden');
    return;
  }

  let path = target;
  try {
    const info = await stat(path);
    if (info.isDirectory()) path = join(path, 'index.html');
    await stat(path);
  } catch {
    res.writeHead(404, { 'Content-Type': 'text/plain; charset=utf-8' }).end('not found');
    return;
  }

  res.writeHead(200, {
    'Content-Type': TYPES[extname(path)] || 'application/octet-stream',
    // 이것이 이 파일의 존재 이유다.
    'Cache-Control': 'no-store, no-cache, must-revalidate',
    Pragma: 'no-cache',
  });
  createReadStream(path).pipe(res);
}).listen(PORT, '0.0.0.0', () => {
  console.log(`dev-server: http://0.0.0.0:${PORT}  (캐시 없음)`);
});
