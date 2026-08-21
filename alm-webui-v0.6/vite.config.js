import { defineConfig } from 'vite';
import { resolve } from 'node:path';

// index.html 과 assets/*.js 는 빌드 대상이 아니다 — 그대로 두면 목업이 단독으로
// 열린다. 여기서는 연동 레이어(src/)만 단일 파일로 묶어 assets/bundle/ 에 떨군다.
// index.html 이 <script type="module"> 로 그 결과물 하나만 불러온다.
export default defineConfig({
  build: {
    outDir: 'assets/bundle',
    emptyOutDir: true,
    // 로봇 온보드는 외부망이 없다. 청크를 쪼개면 file:// 나 단순 정적 서버에서
    // 경로가 어긋나기 쉬워, 한 파일로 합친다.
    lib: {
      entry: resolve(import.meta.dirname, 'src/main.js'),
      formats: ['es'],
      fileName: () => 'main.js',
    },
    // 청크를 쪼개지 않는 이유는 위와 같다 — 산출물 하나만 index.html 이 부른다
    codeSplitting: false,
    target: 'es2022',
    sourcemap: true,
  },
});
