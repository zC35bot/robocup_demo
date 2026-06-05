import { defineConfig } from 'vitepress'
import { withMermaid } from 'vitepress-plugin-mermaid'
import fs from 'node:fs'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

// docs/ 根目录（本文件位于 docs/.vitepress/）
const docsRoot = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..')

// 找出所有形如 "01-启动与架构" 的模块文件夹，按编号排序
function moduleDirs(): string[] {
  return fs
    .readdirSync(docsRoot)
    .filter(
      (name) =>
        /^\d{2}-/.test(name) &&
        fs.statSync(path.join(docsRoot, name)).isDirectory()
    )
    .sort()
}

// 模块内的子篇（排除目录首页 index.md），按 1.1 / 1.2 ... 数字顺序排
function subPages(dir: string): string[] {
  return fs
    .readdirSync(path.join(docsRoot, dir))
    .filter(
      (f) =>
        f.endsWith('.md') &&
        f.toLowerCase() !== 'index.md' &&
        f.toLowerCase() !== 'readme.md'
    )
    .sort((a, b) => a.localeCompare(b, 'zh', { numeric: true }))
}

// "1.1-入口脚本逐行.md" -> "1.1 入口脚本逐行"
function pageLabel(file: string): string {
  return file.replace(/\.md$/, '').replace(/^(\d+(?:\.\d+)?)-/, '$1 ')
}

// "01-启动与架构" -> "01 · 启动与架构"
function groupText(dir: string): string {
  return dir.replace(/^(\d{2})-/, '$1 · ')
}

const mods = moduleDirs()

const sidebar = mods.map((dir) => ({
  text: groupText(dir),
  link: `/${dir}/`, // 模块标题本身即“模块索引”链接，省掉单独一行
  collapsed: true,
  items: subPages(dir).map((f) => ({
    text: pageLabel(f),
    link: `/${dir}/${f.replace(/\.md$/, '')}`,
  })),
}))

// https://vitepress.dev/reference/site-config
// withMermaid 注入 Mermaid 图表渲染支持（```mermaid 代码块）
export default withMermaid(defineConfig({
  lang: 'zh-CN',
  title: 'RoboCup Demo 源码全解',
  description: '人形机器人足球系统的中文逐行讲解文档',
  // 部署路径前缀：
  //  · GitHub Pages（crepveant.github.io/robocup_demo/）→ CI 注入 DOCS_BASE=/robocup_demo/
  //  · Cloudflare Pages（*.pages.dev，根路径）/ 本地 dev → 不设，默认 '/'
  base: process.env.DOCS_BASE || '/',
  cleanUrls: true,
  lastUpdated: true,
  // docs/README.md 里指向仓库根目录 README 的链接在 docs 之外，忽略其死链检查
  ignoreDeadLinks: [/^\.\.\/README(\.md)?$/],
  themeConfig: {
    nav: [
      { text: '首页', link: '/' },
      { text: '开始阅读', link: `/${mods[0]}/` },
      {
        text: '模块',
        items: mods.map((dir) => ({ text: groupText(dir), link: `/${dir}/` })),
      },
    ],
    sidebar,
    outline: { level: [2, 3], label: '本页目录' },
    docFooter: { prev: '上一篇', next: '下一篇' },
    returnToTopLabel: '回到顶部',
    sidebarMenuLabel: '目录',
    darkModeSwitchLabel: '深色模式',
    lastUpdatedText: '最后更新',
    search: {
      provider: 'local',
      options: {
        translations: {
          button: { buttonText: '搜索文档', buttonAriaLabel: '搜索文档' },
          modal: {
            noResultsText: '无法找到相关结果',
            resetButtonTitle: '清除查询条件',
            footer: {
              selectText: '选择',
              navigateText: '切换',
              closeText: '关闭',
            },
          },
        },
      },
    },
  },
  // Mermaid 渲染配置
  // ① htmlLabels:false —— 用 SVG 文本而非 HTML <div> 渲染节点标签。
  //    HTML 标签由浏览器用真实字体异步重排，但 mermaid 用预估字体量高度，二者不一致
  //    会把节点框算矮、裁掉最后一行（尤其中文+长行自动折行时）。SVG 文本由 mermaid
  //    自己折行、自己量高度，完全一致，框总能包住全部文字，永不裁切。
  // ② useMaxWidth:false —— 输出精确像素 width/height，配合 custom.css 的 height:auto 等比缩放。
  mermaid: {
    theme: 'default',
    htmlLabels: false,
    flowchart: { useMaxWidth: false, htmlLabels: false },
    sequence: { useMaxWidth: false },
    state: { useMaxWidth: false },
    class: { useMaxWidth: false },
  },
}))
