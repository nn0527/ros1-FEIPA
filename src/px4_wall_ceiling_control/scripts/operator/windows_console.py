"""Portable Windows entry point; never issues flight commands on startup."""
import argparse
import json
import os
from pathlib import Path
import runpy
import subprocess
import sys
import time
import urllib.request
import urllib.error
import webbrowser


def package_root():
    if getattr(sys, "frozen", False):
        return Path(sys._MEIPASS) / "px4_wall_ceiling_control"
    return Path(__file__).resolve().parents[2]


def settings_root():
    return Path(os.environ.get("LOCALAPPDATA", str(Path.home()))) / "WallConsole"


def configure(path, previous):
    import tkinter as tk
    from tkinter import filedialog, messagebox, ttk
    root = tk.Tk()
    root.title("贴墙测试页面 · 连接设置")
    root.resizable(False, False)
    values = {}
    defaults = dict(host="192.168.50.66", user="pi", auth="key", ssh_port="22", port="8787",
                    key=str(Path.home() / ".ssh/id_ed25519_wallrobot"),
                    known_hosts=str(Path.home() / ".ssh/known_hosts"),
                    container="ros1_noetic", workspace="/root/ros_ws")
    defaults.update(previous)
    tk.Label(root, text="首次设置后保存，下次双击直接打开浏览器。\n支持密钥或用户名密码；密码不写入配置文件。",
             justify="left", padx=16, pady=12).grid(row=0, column=0, columnspan=3)
    fields = [("host", "树莓派 IP"), ("user", "SSH 用户"), ("auth", "认证方式"), ("key", "SSH 私钥"),
              ("known_hosts", "主机指纹文件"), ("ssh_port", "SSH 端口"),
              ("port", "页面端口"), ("container", "Docker 容器"), ("workspace", "容器工作区")]
    for row, (name, label) in enumerate(fields, 1):
        tk.Label(root, text=label, padx=12).grid(row=row, column=0, sticky="w")
        values[name] = tk.StringVar(value=str(defaults[name]))
        if name == "auth":
            ttk.Combobox(root, textvariable=values[name], values=("key", "password"), state="readonly", width=46).grid(row=row, column=1, pady=4)
        else:
            tk.Entry(root, textvariable=values[name], width=48).grid(row=row, column=1, pady=4)
        if name in ("key", "known_hosts"):
            def browse(field=name):
                selected = filedialog.askopenfilename(parent=root, title="选择 " + field)
                if selected:
                    values[field].set(selected)
            tk.Button(root, text="选择", command=browse).grid(row=row, column=2, padx=8)
    result = {}
    def save():
        proposed = {name: value.get().strip() for name, value in values.items()}
        try:
            for name in ("port", "ssh_port"):
                proposed[name] = int(proposed[name])
                if not 1 <= proposed[name] <= 65535:
                    raise ValueError("端口须为 1–65535")
            if not all(proposed[name] for name in ("host", "user", "container", "workspace")):
                raise ValueError("连接信息不能为空")
            if proposed["auth"] == "key" and not Path(proposed["key"]).is_file():
                raise ValueError("私钥文件不存在：" + proposed["key"])
            hosts = Path(proposed["known_hosts"])
            hosts.parent.mkdir(parents=True, exist_ok=True)
            hosts.touch(exist_ok=True)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(proposed, ensure_ascii=False, indent=2), encoding="utf-8")
            result.update(proposed)
            root.destroy()
        except (ValueError, OSError) as error:
            messagebox.showerror("配置未保存", str(error), parent=root)
    tk.Button(root, text="保存并打开测试页面", command=save, padx=20, pady=6).grid(
        row=10, column=0, columnspan=3, pady=14)
    root.mainloop()
    return result


def serve(arguments):
    settings_root().mkdir(parents=True, exist_ok=True)
    if sys.stdout is None:
        sys.stdout = (settings_root() / "portable-server.log").open("a", encoding="utf-8")
    if sys.stderr is None:
        sys.stderr = sys.stdout
    sys.argv = ["wall_console_server.py"] + arguments
    # Import dynamically here so the normal launcher stays lightweight.
    runpy.run_path(str(package_root() / "scripts/operator/wall_console_server.py"), run_name="__main__")


def launch(config, no_browser=False):
    url = "http://127.0.0.1:" + str(config["port"])
    def ready():
        try:
            with urllib.request.urlopen(url + "/api/status", timeout=1) as response:
                status = json.load(response)
            if not status.get("token"):
                raise RuntimeError("页面端口已被其他程序占用")
            expected = dict(ssh_port=int(config["ssh_port"]), container=config["container"],
                            remote_workspace=config["workspace"])
            if status.get("ssh", {}).get("host") != config["user"] + "@" + config["host"] or status.get("connection") != expected:
                raise RuntimeError("该页面端口已有不同连接，请修改页面端口")
            return True
        except (OSError, urllib.error.URLError):
            return False
    if not ready():
        import paramiko
        from tkinter import simpledialog, messagebox
        import base64
        import hashlib
        auth = config.get("auth", "key")
        password = None
        if auth == "password":
            password = simpledialog.askstring("SSH 连接", "请输入 " + config["user"] + "@" + config["host"] + " 的密码（不保存）", show="*")
            if password is None:
                return
        class VerifyHost(paramiko.MissingHostKeyPolicy):
            def missing_host_key(self, client, hostname, key):
                fingerprint = "SHA256:" + base64.b64encode(hashlib.sha256(key.asbytes()).digest()).decode().rstrip("=")
                if not messagebox.askyesno("首次连接：核验主机指纹", hostname + "\n" + fingerprint + "\n\n请与树莓派的主机指纹核对一致后确认。是否信任此主机？"):
                    raise RuntimeError("未确认树莓派主机指纹")
                client.get_host_keys().add(hostname, key.get_name(), key)
                client.save_host_keys(config["known_hosts"])
        probe = paramiko.SSHClient()
        try:
            probe.load_host_keys(config["known_hosts"])
            probe.set_missing_host_key_policy(VerifyHost())
            probe.connect(config["host"], port=int(config["ssh_port"]), username=config["user"],
                          key_filename=config["key"] if auth == "key" else None, password=password,
                          look_for_keys=False, allow_agent=False, timeout=3, auth_timeout=3, banner_timeout=3)
        except (OSError, paramiko.ssh_exception.NoValidConnectionsError):
            pass  # Offline page remains useful; transport retries safely.
        finally:
            probe.close()
        args = ["--ssh-host", config["host"], "--ssh-user", config["user"],
                "--ssh-auth", auth,
                "--ssh-port", str(config["ssh_port"]), "--ssh-key", config["key"],
                "--known-hosts", config["known_hosts"], "--port", str(config["port"]),
                "--container", config["container"], "--remote-workspace", config["workspace"],
                "--settings", str(settings_root() / "flight-settings.json")]
        command = [sys.executable]
        if not getattr(sys, "frozen", False):
            command.append(str(Path(__file__).resolve()))
        command += ["--serve"] + args
        settings_root().mkdir(parents=True, exist_ok=True)
        environment = os.environ.copy()
        # A detached one-file worker needs its own extraction lifetime.
        if getattr(sys, "frozen", False):
            environment["PYINSTALLER_RESET_ENVIRONMENT"] = "1"
        if password is not None:
            environment["WALL_CONSOLE_SSH_PASSWORD"] = password
        with (settings_root() / "portable-server.log").open("ab") as log:
            child = subprocess.Popen(command, stdout=log, stderr=log,
                                     env=environment,
                                     creationflags=subprocess.CREATE_NO_WINDOW)
        for _ in range(100):
            if ready():
                break
            if child.poll() is not None:
                raise RuntimeError("页面启动失败，请查看 " + str(settings_root() / "portable-server.log"))
            time.sleep(.2)
        else:
            raise RuntimeError("页面启动超时，请查看 " + str(settings_root() / "portable-server.log"))
    if not no_browser:
        webbrowser.open(url)


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "--serve":
        serve(sys.argv[2:])
        return
    parser = argparse.ArgumentParser()
    parser.add_argument("--configure", action="store_true")
    parser.add_argument("--no-browser", action="store_true")
    options = parser.parse_args()
    path = settings_root() / "connection.json"
    config = json.loads(path.read_text(encoding="utf-8")) if path.exists() else {}
    if not config or options.configure:
        config = configure(path, config)
    if config:
        launch(config, options.no_browser)


if __name__ == "__main__":
    worker = len(sys.argv) > 1 and sys.argv[1] == "--serve"
    try:
        main()
    except Exception as error:
        if getattr(sys, "frozen", False):
            import traceback
            settings_root().mkdir(parents=True, exist_ok=True)
            (settings_root() / "launcher-error.log").write_text(traceback.format_exc(), encoding="utf-8")
            if worker:
                sys.exit(1)
            from tkinter import messagebox
            messagebox.showerror("贴墙测试页面", str(error))
        else:
            raise
