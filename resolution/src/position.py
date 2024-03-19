#!/usr/bin/env python3

import sqlite3

class PositionLogger:
    def __init__(self, db_name='robot_positions.db'):
        self.conn = sqlite3.connect(db_name)
        self.cursor = self.conn.cursor()
        self.create_table()

    def create_table(self):
        self.cursor.execute('''CREATE TABLE IF NOT EXISTS positions 
                               (id INTEGER PRIMARY KEY, x REAL, y REAL)''')
        self.conn.commit()

    def log_position(self, x, y):
        self.cursor.execute("INSERT INTO positions (x, y) VALUES (?, ?)", (x, y))
        self.conn.commit()
        print("Position enregistrée : x =", x, ", y =", y)

    def close_connection(self):
        self.conn.close()

if __name__ == "__main__":
    logger = PositionLogger()
    # Exemple d'utilisation : enregistrez la position (1, 2)
    logger.log_position(1, 2)
    logger.close_connection()

