# README

## Introduction

本仓库记录了我在遇到一些问题时，所查找到的可行的解决方案，但可能存在归因并不全面的情况，而导致解决方案在特定条件下并不能解决同样的问题。

同时也记录了一些我的学习笔记，比如一些对操作系统、数据库、网络协议的知识汇总和个人理解。

## More Info

1. *How_to_Ask_Questions.md* 是我从[华蟒用户组](https://groups.google.com/g/python-cn)的提问指南中找到的，英文原文为[How To Ask Questions the Smart Way](http://linuxmafia.com/faq/Essays/smart-questions.html)。

## Migration

`Scripts` 目录和 `Config/statusline.sh` 已迁移到 [gwyntoria/skills 的 scripts 目录](https://github.com/gwyntoria/skills/tree/main/scripts)，本仓库不再保留副本。

| 原路径 | 现位置 |
| --- | --- |
| `Scripts/wsl_setup.sh` | [`scripts/wsl_setup.sh`](https://github.com/gwyntoria/skills/blob/main/scripts/wsl_setup.sh) |
| `Scripts/wsl_uninstall.sh` | 并入 [`scripts/wsl_setup.sh`](https://github.com/gwyntoria/skills/blob/main/scripts/wsl_setup.sh) 的 `--uninstall` |
| `Scripts/toria-up.sh` | [`scripts/toria-up.sh`](https://github.com/gwyntoria/skills/blob/main/scripts/toria-up.sh) |
| `Scripts/config_agent.sh` | [`scripts/config_agent.sh`](https://github.com/gwyntoria/skills/blob/main/scripts/config_agent.sh) |
| `Config/statusline.sh` | [`scripts/statusline.sh`](https://github.com/gwyntoria/skills/blob/main/scripts/statusline.sh) |

## Formatting

本仓库使用 `markdownlint-cli2` 和 `textlint` 检查 Markdown 格式、中文排版与技术术语。安装依赖后，对修改过的文件执行：

```bash
npx markdownlint-cli2 --fix --no-globs path/to/file.md
npx textlint --fix path/to/file.md
```

详细规则见 [Markdown 格式化指南](Manual/markdown格式化指南.md)。
