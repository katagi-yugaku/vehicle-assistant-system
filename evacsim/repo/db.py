# db.py
# db.pyがpostgresへの接続を管理するコードを提供
import os
from dataclasses import dataclass
from contextlib import contextmanager
from typing import Iterator, Optional

import psycopg
from psycopg import Connection
from psycopg.rows import dict_row

@dataclass(frozen=True) # frozen=Trueでインスタント生成後の変更を禁止
class DbConfig:
    '''
        PostgreSQLデータベースの接続設定を保持するクラス
    '''
    host:str = "localhost"
    port:int = 5432
    dbname :str = "evacsim"
    user:str ="katagi"
    password:Optional[str] = None

    # 環境変数を使ってDbConfigインスタンスを生成するファクトリメソッド
    @staticmethod
    def from_env() -> "DbConfig":
        return DbConfig(
            host=os.getenv("PGHOST", "localhost"),
            port=int(os.getenv("PGPORT", "5432")),
            dbname=os.getenv("PGDATABASE", "evacsim"),
            user=os.getenv("PGUSER", "katagi"),
            password=os.getenv("PGPASSWORD"),
        )

    def dsn(self) -> str:
        parts = [
            f"host={self.host}",
            f"port={self.port}",
            f"dbname={self.dbname}",
            f"user={self.user}",
        ]
        if self.password:
            parts.append(f"password={self.password}")
        return " ".join(parts)

class Database:
    '''
        接続管理とトランザクションの管理を行うクラス
    '''
    def __init__(self, dsn: str):
        self.dsn = dsn # data source name
    
    @staticmethod
    def from_env() -> "Database":
        return Database(DbConfig.from_env().dsn())

    @contextmanager
    def connection(self) -> Iterator[Connection]:
        conn = psycopg.connect(self.dsn, row_factory=dict_row)
        try:
            yield conn
        finally:
            conn.close()

    @contextmanager
    def transaction(self) -> Iterator[Connection]:
        """
            例外が出たら rollback、正常なら commit
        """
        with self.connection() as conn:
            with conn.transaction():
                yield conn
    

    @contextmanager
    def cursor(self) -> Iterator[psycopg.Cursor]:
        """
        互換: cur だけ欲しい場合
        """
        with self.transaction() as conn:
            with conn.cursor() as cur:
                yield cur

