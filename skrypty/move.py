import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys, select, termios, tty

msg = """
Sterowanie Katamaranem WAM-V
---------------------------
W - do przodu
S - do tyłu
A - obrót w lewo
D - obrót w prawo
Spacja - STOP

UWAGA: Klikaj klawisze pojedynczo.
CTRL+C aby wyjść
"""

class TeleopKatamaran(Node):
    def __init__(self):
        super().__init__('teleop_katamaran')
        
        self.pub_left = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.pub_right = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        print(msg)
        try:
            while rclpy.ok():
                key = self.get_key()
                l_speed = 0.0
                r_speed = 0.0

                if key == 'w':
                    l_speed, r_speed = 50.0, 50.0
                elif key == 's':
                    l_speed, r_speed = -50.0, -50.0
                elif key == 'a':
                    l_speed, r_speed = -35.0, 35.0
                elif key == 'd':
                    l_speed, r_speed = 35.0, -35.0
                elif key == ' ':
                    l_speed, r_speed = 0.0, 0.0
                elif key == '\x03':
                    break

                if key in ['w', 's', 'a', 'd', ' ']:
                    self.pub_left.publish(Float64(data=l_speed))
                    self.pub_right.publish(Float64(data=r_speed))
                    print(f"Polecenie: {key} -> Silniki: L:{l_speed} R:{r_speed}")

        except Exception as e:
            print(f"Błąd: {e}")
        finally:
            self.pub_left.publish(Float64(data=0.0))
            self.pub_right.publish(Float64(data=0.0))

def main():
    rclpy.init()
    node = TeleopKatamaran()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
