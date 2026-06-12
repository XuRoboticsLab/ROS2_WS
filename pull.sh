#!/bin/bash

# 从服务器拉取最新文件到本地
# 忽略 .gitignore 里的内容，以及 push.sh / pull.sh 本身
# 本地独有的文件不会被删除

REMOTE="xrl_local:/home/xuroboticslab/wbcd/v6"
LOCAL="$(cd "$(dirname "$0")" && pwd)"

EXCLUDES=(
  --exclude 'push.sh'
  --exclude 'pull.sh'
)

if [ -f "$LOCAL/.gitignore" ]; then
  rsync -avz --filter=':- .gitignore' "${EXCLUDES[@]}" "$REMOTE/" "$LOCAL/"
else
  rsync -avz "${EXCLUDES[@]}" "$REMOTE/" "$LOCAL/"
fi

echo "✅ 拉取完成：$REMOTE -> $LOCAL"
