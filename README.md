# 冉梦佳的个人博客

我的个人网站，用来记录学习笔记、项目实践和生活日常。

在线访问：<https://ranmengjia.github.io/>

## 本站内容

- **Blog**：ROS2 学习笔记、Python 踩坑记录、无人机巡检实践等
- **Projects**：我的项目文档与作品说明
- **Vibe**：短记录与生活碎片
- **About**：关于我和友情链接

## 技术栈

- [Astro](https://astro.build/) 静态站点 + Tailwind CSS + Pagefind 全文搜索
- 内容使用 Markdown / MDX 管理
- GitHub Actions 自动构建，部署在 GitHub Pages

## 本地运行

环境要求：[Bun](https://bun.sh/)、Python 3（构建时用于生成字体子集）

```bash
# 安装 Python 依赖（字体子集化）
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip fonttools brotli

# 安装依赖
bun install

# 启动开发服务器
bun run dev

# 构建生产版本
bun run build
```

新建内容：

```bash
bun run post:new my-first-post    # 新博客文章
bun run project:new my-project    # 新项目文档
bun run vibe:new today-cloud      # 新短记录
```

## 致谢

本站基于开源模板 [dodolalorc/astro-navfolio](https://github.com/dodolalorc/astro-navfolio) 二次开发而来，感谢原作者的开源分享。
