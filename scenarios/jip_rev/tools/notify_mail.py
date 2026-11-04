#!/usr/bin/env python3
"""
実験結果をメールで送る(Gmail の SMTP。送り方は ~/test_gmail.py と同じ)。

    python tools/notify_mail.py --subject "..." --body-file body.txt --attach a.png b.csv [--dry-run]

設定は ~/.config/evac-mail/gmail.ini(権限 600)から読む:
    [smtp]
    user = 送信に使う Gmail アドレス
    app_password = アプリパスワード
    to = 受信するアドレス

- 設定ファイルがない、または --dry-run のときは送らずに、件名・本文・添付名を出力する。
- 送信に失敗しても終了コードは 0(集計や保存の結果を変えないため)。失敗は標準エラーに出す。
- 設定ファイルの中身はログに出さない。
"""
from __future__ import annotations

import argparse
import configparser
import mimetypes
import os
import smtplib
import stat
import sys
from email.message import EmailMessage
from pathlib import Path

CONFIG = Path(os.environ.get("EVAC_MAIL_CONFIG", "~/.config/evac-mail/gmail.ini")).expanduser()
MAX_TOTAL = 20 * 1024 * 1024


def choose_attachments(paths):
    files = [p for p in paths if p.is_file()]
    if sum(p.stat().st_size for p in files) <= MAX_TOTAL:
        return files
    out, total = [], 0          # 20 MB を超える場合は PNG / CSV / MD だけにする
    for p in files:
        if p.suffix.lower() in (".png", ".csv", ".md") and total + p.stat().st_size <= MAX_TOTAL:
            out.append(p)
            total += p.stat().st_size
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--subject", required=True)
    ap.add_argument("--body-file", required=True)
    ap.add_argument("--attach", nargs="*", default=[])
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    body = Path(args.body_file).read_text()
    attachments = choose_attachments([Path(a) for a in args.attach])

    if args.dry_run or not CONFIG.is_file():
        why = "--dry-run" if args.dry_run else f"{CONFIG} がない"
        print(f"[mail] 送信をスキップ({why})")
        print(f"[mail] Subject: {args.subject}")
        print("[mail] Attachments: " + ", ".join(p.name for p in attachments))
        print("[mail] ---- body ----\n" + body + "\n[mail] --------------")
        return 0

    if stat.S_IMODE(CONFIG.stat().st_mode) & 0o077:
        print(f"[mail] WARNING: {CONFIG} の権限を 600 にしてください", file=sys.stderr)
    cp = configparser.ConfigParser()
    cp.read(CONFIG)
    try:
        user, password, to = cp["smtp"]["user"], cp["smtp"]["app_password"], cp["smtp"]["to"]
    except KeyError as exc:
        print(f"[mail] ERROR: {CONFIG} の [smtp] に {exc} がありません", file=sys.stderr)
        return 0

    msg = EmailMessage()
    msg["Subject"], msg["From"], msg["To"] = args.subject, user, to
    msg.set_content(body)
    for p in attachments:
        ctype, _ = mimetypes.guess_type(p.name)
        maintype, subtype = (ctype or "application/octet-stream").split("/", 1)
        msg.add_attachment(p.read_bytes(), maintype=maintype, subtype=subtype, filename=p.name)
    try:
        with smtplib.SMTP("smtp.gmail.com", 587, timeout=60) as s:
            s.starttls()
            s.login(user, password)
            s.send_message(msg)
        print(f"[mail] sent: {args.subject} ({len(attachments)} attachments)")
    except Exception as exc:
        print(f"[mail] ERROR: send failed: {exc.__class__.__name__}: {exc}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
