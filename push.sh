#!/bin/bash

# 把本地文件推送到服务器
# 忽略 .gitignore 里的内容，以及 push.sh / pull.sh 本身
# 服务器上独有的文件不会被删除

REMOTE="xrl_local:/home/xuroboticslab/wbcd/v1"
LOCAL="$(cd "$(dirname "$0")" && pwd)"

EXCLUDES=(
  --exclude 'push.sh'
  --exclude 'pull.sh'
)

if [ -f "$LOCAL/.gitignore" ]; then
  rsync -avz --filter=':- .gitignore' "${EXCLUDES[@]}" "$LOCAL/" "$REMOTE/"
else
  rsync -avz "${EXCLUDES[@]}" "$LOCAL/" "$REMOTE/"
fi

echo "✅ 推送完成：$LOCAL -> $REMOTE"
