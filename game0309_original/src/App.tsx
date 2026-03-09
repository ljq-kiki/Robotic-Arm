function App() {
  return (
    <div className="min-h-screen bg-slate-950 text-slate-50 flex items-center justify-center px-4">
      <div className="w-full max-w-md rounded-2xl border border-slate-800 bg-slate-900/60 p-8 shadow-xl shadow-black/40">
        <p className="text-xs font-semibold uppercase tracking-[0.2em] text-sky-400 mb-3">
          Lion Model Game
        </p>
        <h1 className="text-2xl font-semibold tracking-tight sm:text-3xl">
          Vite + React + Tailwind CSS
        </h1>
        <p className="mt-3 text-sm text-slate-400">
          前端项目已经初始化完成。现在你可以开始编写页面、组件和业务逻辑，
          并使用 Tailwind 的原子类快速搭建 UI。
        </p>
        <div className="mt-6 flex flex-wrap items-center gap-3">
          <button
            type="button"
            className="inline-flex items-center justify-center rounded-lg bg-sky-500 px-4 py-2 text-sm font-medium text-slate-950 shadow-sm transition hover:bg-sky-400 focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-sky-400 focus-visible:ring-offset-2 focus-visible:ring-offset-slate-950"
          >
            开始开发
          </button>
          <span className="text-xs text-slate-500">
            运行 <code className="rounded bg-slate-800 px-1.5 py-0.5 font-mono text-[0.7rem]">npm run dev</code> 打开本地开发环境
          </span>
        </div>
      </div>
    </div>
  )
}

export default App
