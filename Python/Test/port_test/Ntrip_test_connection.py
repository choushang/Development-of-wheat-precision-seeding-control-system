# 用于测试Ntrip是否可用

import socket
import base64

class NtripClient:
    def __init__(self, host, port, mountpoint, user, password):
        self.host = host
        self.port = port
        self.mountpoint = mountpoint
        self.user = user
        self.password = password

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.host, self.port))
        self.send_request()

    def send_request(self):
        credentials = f"{self.user}:{self.password}"
        credentials_encoded = base64.b64encode(credentials.encode()).decode()
        request = f"GET /{self.mountpoint} HTTP/1.1\r\n"
        request += f"Host: {self.host}\r\n"
        request += f"Authorization: Basic {credentials_encoded}\r\n"
        request += "Ntrip-Version: Ntrip/2.0\r\n"
        request += "\r\n"
        self.sock.send(request.encode())

    def receive_data(self):
        try:
            # 接收数据，这里仅接收前4096字节作为示例
            data = self.sock.recv(4096)
            print(data.decode())
        except KeyboardInterrupt:
            self.sock.close()

    def close_connection(self):
        self.sock.close()

if __name__ == '__main__':
    ntrip_client = NtripClient("120.253.239.161", 8002, "RTCM33_GRCEJ", "cavm174", "8uh087n8")
    ntrip_client.connect()
    ntrip_client.receive_data()  # 接收并打印数据
    ntrip_client.close_connection()  # 断开连接
