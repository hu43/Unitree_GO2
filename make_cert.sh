#!/bin/bash
# 生成 web 面板的自签 TLS 证书（Web Bluetooth 要求页面处于安全上下文，
# 也就是必须走 https；自签证书点过警告后 Chrome 仍会把它当安全上下文）。
#
# 用法：./make_cert.sh
# 产物：certs/server.crt / certs/server.key（有效期 10 年）
# 换 IP 后重跑一次即可，SAN 会按当前网卡地址重新收集。
set -e
cd "$(dirname "$0")"
mkdir -p certs

IPS=$(hostname -I 2>/dev/null | tr ' ' '\n' | grep -E '^[0-9.]+$' || true)
# 固定补上热点网关和回环，以及常见调试地址
SAN="DNS:localhost,IP:127.0.0.1,IP:192.168.124.1,IP:192.168.123.18"
for ip in $IPS; do
  case ",$SAN," in
    *",IP:$ip,"*) ;;
    *) SAN="$SAN,IP:$ip" ;;
  esac
done

echo "SAN = $SAN"

openssl req -x509 -newkey rsa:2048 -nodes \
  -keyout certs/server.key -out certs/server.crt \
  -days 3650 -subj "/CN=Unitree-Go2" \
  -addext "subjectAltName=$SAN" 2>/dev/null

chmod 600 certs/server.key
chmod 644 certs/server.crt
echo "已生成: $(pwd)/certs/server.crt"
openssl x509 -in certs/server.crt -noout -subject -dates -ext subjectAltName
