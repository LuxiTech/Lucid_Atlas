#!/usr/bin/env python3
import argparse
import select
import socket
import socketserver


class ProxyServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, address, handler, target):
        self.target = target
        super().__init__(address, handler)


class ProxyHandler(socketserver.BaseRequestHandler):
    def handle(self):
        upstream = socket.create_connection(self.server.target, timeout=5.0)
        sockets = [self.request, upstream]
        try:
            for sock in sockets:
                sock.setblocking(False)
            while True:
                readable, _, errored = select.select(sockets, [], sockets, 30.0)
                if errored:
                    break
                if not readable:
                    continue
                for sock in readable:
                    data = sock.recv(65536)
                    if not data:
                        return
                    peer = upstream if sock is self.request else self.request
                    peer.sendall(data)
        finally:
            upstream.close()


def main():
    parser = argparse.ArgumentParser(description="Forward a local TCP port to the Luxi Atlas web API.")
    parser.add_argument("--listen", default="0.0.0.0")
    parser.add_argument("--listen-port", type=int, default=80)
    parser.add_argument("--target-host", default="127.0.0.1")
    parser.add_argument("--target-port", type=int, default=8082)
    args = parser.parse_args()

    server = ProxyServer(
        (args.listen, args.listen_port),
        ProxyHandler,
        (args.target_host, args.target_port),
    )
    print(
        f"Forwarding {args.listen}:{args.listen_port} "
        f"to {args.target_host}:{args.target_port}",
        flush=True,
    )
    server.serve_forever()


if __name__ == "__main__":
    main()
