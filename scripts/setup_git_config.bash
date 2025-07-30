#!/bin/bash

# Git和SSH自动配置脚本
# 用法: ./setup_git_config.bash

set -e  # 遇到错误时退出

echo "=== Git和SSH自动配置脚本 ==="

# 检查是否已有SSH密钥
if [ -f ~/.ssh/id_rsa ]; then
    echo "检测到已存在SSH密钥 (~/.ssh/id_rsa)"
    read -p "是否要重新生成? (y/N): " regenerate
    if [[ $regenerate =~ ^[Yy]$ ]]; then
        rm -f ~/.ssh/id_rsa ~/.ssh/id_rsa.pub
        echo "已删除旧密钥"
    else
        echo "跳过SSH密钥生成"
        skip_ssh=true
    fi
fi

# 生成SSH密钥
if [ "$skip_ssh" != true ]; then
    read -p "请输入邮箱地址: " email
    echo "正在生成SSH密钥..."
    ssh-keygen -t rsa -b 4096 -C "$email" -f ~/.ssh/id_rsa -N ""
    echo "SSH密钥已生成"
    
    # 启动ssh-agent并添加密钥
    eval "$(ssh-agent -s)"
    ssh-add ~/.ssh/id_rsa
    echo "SSH密钥已添加到ssh-agent"
fi

# 配置Git用户信息
echo -e "\n=== 配置Git用户信息 ==="
current_name=$(git config --global user.name 2>/dev/null || echo "")
current_email=$(git config --global user.email 2>/dev/null || echo "")

if [ -n "$current_name" ]; then
    echo "当前Git用户名: $current_name"
    read -p "是否要更改? (y/N): " change_name
    if [[ $change_name =~ ^[Yy]$ ]]; then
        read -p "请输入新的用户名: " username
        git config --global user.name "$username"
    fi
else
    read -p "请输入Git用户名: " username
    git config --global user.name "$username"
fi

if [ -n "$current_email" ]; then
    echo "当前Git邮箱: $current_email"
    read -p "是否要更改? (y/N): " change_email
    if [[ $change_email =~ ^[Yy]$ ]]; then
        read -p "请输入新的邮箱: " git_email
        git config --global user.email "$git_email"
    fi
else
    if [ -z "$email" ]; then
        read -p "请输入Git邮箱: " git_email
    else
        git_email="$email"
    fi
    git config --global user.email "$git_email"
fi

# 显示公钥
echo -e "\n=== 配置完成 ==="
echo "Git配置:"
echo "  用户名: $(git config --global user.name)"
echo "  邮箱: $(git config --global user.email)"

if [ -f ~/.ssh/id_rsa.pub ]; then
    echo -e "\nSSH公钥内容 (复制到GitHub/GitLab):"
    echo "----------------------------------------"
    cat ~/.ssh/id_rsa.pub
    echo "----------------------------------------"
fi

echo -e "\n配置完成! 请将上述公钥添加到你的Git服务提供商账户中。"