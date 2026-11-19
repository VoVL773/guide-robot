# GitHub 上传指南

本文只说明如何上传；本次文档整理没有创建远程仓库、提交或推送代码。

## 提交范围

建议提交根 README、`docs/`、`guide_ws/src/guide_voice/`、`web_monitor/` 和 `.gitignore`。

下列本机内容已配置忽略：

- colcon 的 `build/`、`install/`、`log/`；
- `.codex-backups/`、Python 缓存、运行日志与 PID 文件；
- `.vscode/`、`learn/`、`arm_demo/`；
- `.env`、`.env.*` 和 `*.local.yaml` / `*.local.yml` 本地配置（`.env.example` 可提交）。

本次整理将此前跟踪的 `guide_ws/.codex-backups/` 从 Git 索引移除，磁盘文件保留。**已存在的历史提交仍可能含有这些文件**，`.gitignore` 和索引移除都不会清除历史。

## 发布前核对

- 当前代码仍含原现场地址：底盘 `192.168.11.10:9090`、平台与 SDK 路径，以及模型服务 `120.48.75.178:7006`。按公开范围确认是否保留；使用者应覆盖为自己的配置。
- 检查地点名称、`web_monitor/map.png`、示意图素材及其 JSON 数据是否允许公开。文档整理未删除这些项目资源。
- 包元数据声明 `Apache-2.0`，仓库未提供完整许可证文本。确认权属后补充适用的 `LICENSE`；不要替第三方 SDK 或模型授予许可。
- 不要上传 API Key、平台凭据、私有日志和语音录像。新增忽略规则无法移除既有历史中的内容。

从仓库根目录查看当前状态、文件范围及相关配置出现位置：

```bash
git status --short
git diff --stat
git diff --cached --stat
git ls-files
rg -n --hidden -g '!.git/**' -g '!.codex-backups/**' \
  '120\.48\.75\.178|192\.168\.11\.10|/home/nvidia|api[_-]?key|token|password' \
  README.md docs guide_ws/src web_monitor
```

搜索会匹配正常的参数名和示例，不等于发现泄露，也不是完整的历史凭据扫描。若旧提交中出现真实凭据，应先轮换凭据，再按需要清理历史；仅在最新提交中删除文件不能消除历史副本。

## 提交本地整理结果

确认状态后暂存项目文件并检查差异：

```bash
git add README.md .gitignore docs guide_ws/src/guide_voice web_monitor
git diff --cached --stat
git diff --cached --check
git status --short
```

本次备份目录的索引删除已暂存，会随下一次提交生效。确认提交内容后执行：

```bash
git commit -m "docs: 整理项目说明、部署步骤与 GitHub 上传指南"
```

## 首次推送

先在 GitHub 创建空仓库，不勾选自动生成 README、许可证和 `.gitignore`，避免与本地已有历史冲突。可先创建为私有仓库，核对公开内容后再调整可见性。

查看当前分支和远程配置：

```bash
git branch --show-current
git remote -v
```

如果尚无 `origin`，把下面 URL 替换为自己的仓库地址：

```bash
git remote add origin https://github.com/YOUR_USERNAME/guide_robot.git
git push -u origin HEAD
```

`HEAD` 表示推送当前分支，不要求本地分支必须叫 `main`。如果已有 `origin`，先确认地址，正确时直接执行推送；地址错误时再修改：

```bash
git remote set-url origin https://github.com/YOUR_USERNAME/guide_robot.git
git push -u origin HEAD
```

若远程仓库已有提交，先获取并检查分支差异，再合并或变基，不要直接强制推送覆盖已有内容。GitHub HTTPS 登录请使用受支持的凭据方式，或使用已配置好的 SSH 地址。

上传后在 GitHub 打开 README，确认 Mermaid 图可以渲染、文档链接正常，且文件列表中没有构建产物、备份和运行日志。
