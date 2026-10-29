"""后台 SSH 连接线程，避免网络操作阻塞图形界面。"""
import json
from pathlib import Path
import queue
import select
import shlex
import socketserver
import sys
import threading
import time

import paramiko
from PySide6.QtCore import QThread, Signal


class VncForward(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


class VncHandler(socketserver.BaseRequestHandler):
    def handle(self):
        """@brief 仅转发本机 VNC 到树莓派回环 5900，通过已有 SSH 加密。"""
        channel=None
        try:
            channel=self.server.transport.open_channel('direct-tcpip',('127.0.0.1',5900),
                                                       self.request.getpeername(),timeout=5)
            while not self.server.stopping.is_set():
                ready,_,_=select.select([self.request,channel],[],[],.2)
                for source in ready:
                    data=source.recv(65536)
                    if not data:
                        return
                    (channel if source is self.request else self.request).sendall(data)
        except (OSError,paramiko.SSHException):
            pass
        finally:
            if channel:
                channel.close()


class Connection(QThread):
    message = Signal(dict)
    status = Signal(str)
    failure = Signal(str)

    def __init__(self, host, port, user, password):
        """@brief 保存单次连接参数，密码仅在内存中用于 SSH 和 sudo。"""
        super().__init__()
        self.host, self.port, self.user = host, port, user
        self.password = password
        self.pending = queue.Queue(maxsize=50)
        self.stopping = threading.Event()
        self.clock_offset = 0.
        self.ready = False
        self.vnc_port = None

    def send(self, command):
        """@brief 非阻塞提交操作，遥控请求携带固定有效期，线程不延长期限。"""
        command = dict(command)
        if command.get('action') in ('motion', 'arm', 'servo', 'navigation_enable'):
            command['deadline'] = time.time() + self.clock_offset + .2
        elif command.get('action') in ('navigation_initial_pose','navigation_goal'):
            command['deadline'] = time.time() + self.clock_offset + 1.0
        try:
            self.pending.put_nowait(command)
        except queue.Full:
            self.stopping.set()

    def stop(self):
        """@brief 请求撤销连接，由工作线程负责停止远端会话并关闭通道。"""
        self.stopping.set()

    def upload(self, client):
        """@brief 上传随 exe 附带的网关，不修改 ROS 包或底盘固件。"""
        root = Path(getattr(sys, '_MEIPASS', Path(__file__).resolve().parents[1]))
        sftp = client.open_sftp()
        try:
            home = sftp.normalize('.')
            self.remote_home = home
            base = home + '/ros2_ws/robot_console'
            # 路径按登录账号的 home 解析，不能由界面拼接 shell 文本。
            for directory in (home+'/ros2_ws', base, base+'/pi', base+'/shared'):
                try:
                    sftp.stat(directory)
                except OSError:
                    sftp.mkdir(directory)
            for relative in ('pi/gateway.py', 'pi/lidar.rviz', 'pi/navigation.rviz', 'pi/navigation.launch.py', 'pi/navigation_tf.py', 'pi/navigation_control.py', 'pi/navigation_scene.py', 'pi/k230_bridge.py', 'pi/chassis_node.py', 'pi/heading_control.py', 'pi/wheel_odometry.py', 'pi/parameter_store.py', 'shared/protocol.py', 'shared/results.py', 'shared/navigation_geometry.py'):
                target = base + '/' + relative
                temporary = target + '.upload'
                sftp.put(str(root / relative), temporary)
                sftp.posix_rename(temporary, target)
            # 初次部署参数文件；后续连接保留树莓派上已调好的参数。
            config = base + '/pi/chassis_params.yaml'
            try:
                sftp.stat(config)
            except OSError:
                sftp.put(str(root / 'pi/chassis_params.yaml'), config)
        finally:
            sftp.close()

    def run(self):
        """@brief 校验主机密钥、同步网关并通过 SSH 通道收发 JSON 数据。"""
        client = paramiko.SSHClient()
        channel = None
        forward = None
        try:
            known = Path.home()/'.ssh'/'known_hosts'
            if known.exists():
                client.load_host_keys(str(known))
            client.set_missing_host_key_policy(paramiko.RejectPolicy())
            self.status.emit('正在连接 SSH…')
            client.connect(self.host, port=self.port, username=self.user,
                           password=self.password, timeout=8, banner_timeout=8,
                           auth_timeout=10, look_for_keys=False, allow_agent=False)
            client.get_transport().set_keepalive(5)
            if self.stopping.is_set():
                return
            self.status.emit('正在同步网关…')
            self.upload(client)
            # 程序名、容器名和容器内路径固定，避免 shell 注入。
            self.status.emit('正在启动 ROS 网关…')
            # 外层命令使用统一引号拼接，所有内层参数均为固定文本。
            home=self.remote_home
            copy_auth=('if [ -f '+shlex.quote(home+'/.Xauthority')+' ]; then cp -- '+
                       shlex.quote(home+'/.Xauthority')+' '+shlex.quote(home+'/ros2_ws/robot_console/.Xauthority')+
                       '; chmod 600 '+shlex.quote(home+'/ros2_ws/robot_console/.Xauthority')+'; fi; ')
            inner = (copy_auth+'docker start ros2-humble >/dev/null && exec docker exec -i '
                     '-e XAUTHLOCALHOSTNAME="$(hostname)" ros2-humble bash -c ' + shlex.quote(
                         'source /opt/ros/humble/setup.bash; '
                         'source /workspace/ros2_ws/install/setup.bash; '
                         'exec python3 -u /workspace/ros2_ws/robot_console/pi/gateway.py'))
            # 仅在专属密码提示出现时发送密码，兼容免密码 sudo 和已有凭据。
            command = "sudo -k -S -p '__ROBOT_SUDO_PASSWORD__\n' sh -c " + shlex.quote(inner)
            transport = client.get_transport()
            channel = transport.open_session(timeout=8)
            channel.exec_command(command)
            password_sent = False
            buffer = bytearray()
            started = time.monotonic()
            last_receive = started
            stderr = bytearray()
            while not self.stopping.is_set():
                if channel.recv_ready():
                    chunk = channel.recv(65536)
                    if not chunk:
                        break
                    buffer.extend(chunk)
                    if len(buffer) > 8000000:
                        raise ValueError('网关消息超过大小限制')
                    while b'\n' in buffer:
                        raw, _, buffer = buffer.partition(b'\n')
                        try:
                            data = json.loads(raw)
                        except ValueError:
                            self.message.emit({'type':'log', 'text':raw.decode('utf-8','replace')[:2000]})
                            continue
                        last_receive = time.monotonic()
                        if data.get('type') == 'hello':
                            if data.get('version') != 1:
                                raise ValueError('网关版本不匹配')
                            self.clock_offset = data['server_time'] - time.time()
                            self.ready = True
                            try:
                                forward=VncForward(('127.0.0.1',0),VncHandler)
                                forward.transport=transport
                                forward.stopping=self.stopping
                                self.vnc_port=forward.server_address[1]
                                threading.Thread(target=forward.serve_forever,daemon=True).start()
                            except OSError as error:
                                self.message.emit({'type':'log','text':f'VNC 转发未建立：{error}'})
                            self.status.emit('已连接 · SSH 加密通道')
                            self.send({'action':'ports'})
                        self.message.emit(data)
                if channel.recv_stderr_ready():
                    stderr.extend(channel.recv_stderr(4096))
                    if b'__ROBOT_SUDO_PASSWORD__' in stderr:
                        if password_sent:
                            raise PermissionError('sudo 认证失败，请核对密码和该账号的 sudo 权限')
                        channel.sendall((self.password+'\n').encode('utf-8'))
                        self.password = ''
                        password_sent = True
                        stderr[:] = stderr.replace(b'__ROBOT_SUDO_PASSWORD__\n',b'')
                    if b'\n' in stderr or len(stderr) > 4096:
                        self.message.emit({'type':'log', 'text':stderr.decode('utf-8','replace')})
                        stderr.clear()
                if self.ready:
                    for _ in range(20):
                        try:
                            item = self.pending.get_nowait()
                        except queue.Empty:
                            break
                        # 堵塞后的旧遥控不进入网络，禁能操作仍然允许发送。
                        if item.get('action') in ('motion', 'arm', 'servo', 'navigation_enable', 'navigation_initial_pose','navigation_goal') and item['deadline'] <= time.time()+self.clock_offset:
                            continue
                        channel.sendall((json.dumps(item)+'\n').encode('utf-8'))
                if channel.exit_status_ready():
                    raise RuntimeError('远端网关已退出：'+stderr.decode('utf-8','replace'))
                if not self.ready and time.monotonic()-started > 25:
                    raise TimeoutError('网关未就绪，请检查 sudo、Docker 和容器路径')
                if self.ready and time.monotonic()-last_receive > 3:
                    raise TimeoutError('网关数据超时，已断开控制')
                time.sleep(.01)
        except Exception as error:
            if not self.stopping.is_set():
                hint = str(error)
                if isinstance(error, paramiko.SSHException) and 'known_hosts' in hint:
                    hint += '\n请先在 Windows 终端使用 ssh 登录，核对并保存树莓派主机指纹。'
                self.failure.emit(hint)
        finally:
            self.ready = False
            self.password = ''
            if forward:
                forward.shutdown()
                forward.server_close()
            self.vnc_port = None
            if channel:
                try:
                    channel.settimeout(.5)
                    channel.sendall(b'{"action":"disarm"}\n{"action":"shutdown"}\n')
                    channel.shutdown_write()
                except Exception:
                    pass
                channel.close()
            client.close()
            self.status.emit('未连接')
