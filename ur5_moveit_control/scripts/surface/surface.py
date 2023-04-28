import sys
import pygame
import downcontrol  
import robotcontrol
import time
from time import sleep
import os

class settings():
    """存储界面所有设置的类"""

    def __init__(self):
        """初始化通用参数设置"""
        # 屏幕设置
        self.screen_width = 1200
        self.screen_height = 800
        self.bg_color = (230, 230, 230)

        # 飞船设置
        self.ship_limit = 3

class imageslogo():
    """公司logo图片"""
    def __init__(self, screen):
        """设置初始位置"""
        self.screen = screen

        # 初始化图片并获取其外接矩形
        self.image = pygame.image.load('/home/ts/catkin_ws/src/ur5_moveit_control/scripts/images/logo2.jpg')
        #self.image = pygame.image.load('/home/ts/catkin_ws/src/ur5_moveit_control/scripts/images/ship.bmp')
        self.rect = self.image.get_rect()
        self.screen_rect = screen.get_rect()

        # 将公司logo放在屏幕左上角
        self.rect.centerx = self.screen_rect.centerx
        self.rect.bottom = self.screen_rect.bottom

    def blitme(self):
        """在指定位置绘制图片"""
        self.screen.blit(self.image, self.rect)

class Button():
    
    def __init__(self, ai_settings, screen, msg):
        """初始化按钮属性"""
        self.screen = screen
        self.screen_rect = screen.get_rect()

        # 设置按钮的尺寸和其他属性
        self.width, self.height = 210, 50
        self.button_color = (0, 255, 0)
        self.text_color = (255, 255, 255)
        self.font = pygame.font.SysFont(None, 48)

        # 创建按钮的rect对象，并使其居中
        self.rect = pygame.Rect(0, 0, self.width, self.height)
        # self.rect.midleft = self.screen_rect.midleft
        self.rect.x = 50
        self.rect.y = 50


        # 按钮的标签只需创建一次
        self.prep_msg(msg)

    def prep_msg(self, msg):
        """将msg渲染为图像,并使其在按钮上居中"""
        self.msg_image = self.font.render(msg, True, self.text_color, self.button_color)
        self.msg_image_rect = self.msg_image.get_rect()
        # self.msg_image_rect.center = self.rect.center
        # self.msg_image_rect.left = self.rect.left
        self.msg_image_rect.x = self.rect.x
        self.msg_image_rect.y = self.rect.y + 8

    def draw_button(self):
        """绘制一个用颜色填充的按钮,再绘制文本"""
        self.screen.fill(self.button_color, self.rect)
        self.screen.blit(self.msg_image, self.msg_image_rect)


class GrabButton():
    
    def __init__(self, ai_settings, screen, msg):
        """初始化按钮属性"""
        self.screen = screen
        self.screen_rect = screen.get_rect()

        # 设置按钮的尺寸和其他属性
        self.width, self.height = 210, 50
        self.button_color = (0, 255, 0)
        self.text_color = (255, 255, 255)
        self.font = pygame.font.SysFont(None, 48)

        # 创建按钮的rect对象，并使其居中
        self.rect = pygame.Rect(0, 0, self.width, self.height)
        # self.rect.center = self.screen_rect.center 
        self.rect.x = 50
        self.rect.y = 450      

        # 按钮的标签只需创建一次
        self.prep_msg(msg)

    def prep_msg(self, msg):
        """将msg渲染为图像,并使其在按钮上居中"""
        self.msg_image = self.font.render(msg, True, self.text_color, self.button_color)
        self.msg_image_rect = self.msg_image.get_rect()
        self.msg_image_rect.x = self.rect.x
        self.msg_image_rect.y = self.rect.y + 8
        

    def draw_button(self):
        """绘制一个用颜色填充的按钮,再绘制文本"""
        self.screen.fill(self.button_color, self.rect)
        self.screen.blit(self.msg_image, self.msg_image_rect)


class EndButton():
    
    def __init__(self, ai_settings, screen, msg):
        """初始化按钮属性"""
        self.screen = screen
        self.screen_rect = screen.get_rect()

        # 设置按钮的尺寸和其他属性
        self.width, self.height = 210, 50
        self.button_color = (0, 255, 0)
        self.text_color = (255, 255, 255)
        self.font = pygame.font.SysFont(None, 48)

        # 创建按钮的rect对象，并使其居中
        self.rect = pygame.Rect(0, 0, self.width, self.height)
        self.rect.x = 50
        self.rect.y = 250 

        # 按钮的标签只需创建一次
        self.prep_msg(msg)

    def prep_msg(self, msg):
        """将msg渲染为图像,并使其在按钮上居中"""
        self.msg_image = self.font.render(msg, True, self.text_color, self.button_color)
        self.msg_image_rect = self.msg_image.get_rect()
        self.msg_image_rect.x = self.rect.x
        self.msg_image_rect.y = self.rect.y + 8

    def draw_button(self):
        """绘制一个用颜色填充的按钮,再绘制文本"""
        self.screen.fill(self.button_color, self.rect)
        self.screen.blit(self.msg_image, self.msg_image_rect)


class LinkButton():
    
    def __init__(self, ai_settings, screen, msg):
        """初始化按钮属性"""
        self.screen = screen
        self.screen_rect = screen.get_rect()

        # 设置按钮的尺寸和其他属性
        self.width, self.height = 210, 50
        self.button_color = (0, 255, 0)
        self.text_color = (255, 255, 255)
        self.font = pygame.font.SysFont(None, 48)

        # 创建按钮的rect对象，并使其居中
        self.rect = pygame.Rect(0, 0, self.width, self.height)
        self.rect.x = 50
        self.rect.y = 650 

        # 按钮的标签只需创建一次
        self.prep_msg(msg)

    def prep_msg(self, msg):
        """将msg渲染为图像,并使其在按钮上居中"""
        self.msg_image = self.font.render(msg, True, self.text_color, self.button_color)
        self.msg_image_rect = self.msg_image.get_rect()
        self.msg_image_rect.x = self.rect.x
        self.msg_image_rect.y = self.rect.y + 8

    def draw_button(self):
        """绘制一个用颜色填充的按钮,再绘制文本"""
        self.screen.fill(self.button_color, self.rect)
        self.screen.blit(self.msg_image, self.msg_image_rect)

class SampleButton():
    
    def __init__(self, ai_settings, screen, msg):
        """初始化按钮属性"""
        self.screen = screen
        self.screen_rect = screen.get_rect()

        # 设置按钮的尺寸和其他属性
        self.width, self.height = 210, 50
        self.button_color = (0, 255, 0)
        self.text_color = (255, 255, 255)
        self.font = pygame.font.SysFont(None, 48)

        # 创建按钮的rect对象，并使其居中
        self.rect = pygame.Rect(0, 0, self.width, self.height)
        self.rect.x = 380
        self.rect.y = 650 

        # 按钮的标签只需创建一次
        self.prep_msg(msg)

    def prep_msg(self, msg):
        """将msg渲染为图像,并使其在按钮上居中"""
        self.msg_image = self.font.render(msg, True, self.text_color, self.button_color)
        self.msg_image_rect = self.msg_image.get_rect()
        self.msg_image_rect.x = self.rect.x
        self.msg_image_rect.y = self.rect.y + 8

    def draw_button(self):
        """绘制一个用颜色填充的按钮,再绘制文本"""
        self.screen.fill(self.button_color, self.rect)
        self.screen.blit(self.msg_image, self.msg_image_rect)

def update_screen(ai_settings, screen, ship, play_button, Grab_button, End_button, Link_button, Sample_button):
    """更新屏幕上的图片,并切换到新屏幕"""
    # 每次循环时都重绘屏幕
    screen.fill(ai_settings.bg_color)
    ship.blitme()

    play_button.draw_button()
    Grab_button.draw_button()
    End_button.draw_button()
    Link_button.draw_button()
    Sample_button.draw_button()

    # 让最近绘制的屏幕可见
    pygame.display.flip()


def check_play_button(stats, play_button, Grab_button, End_button, Link_button, Sample_button, mouse_x, mouse_y):
    """单击play按键时项目开始"""
    if play_button.rect.collidepoint(mouse_x, mouse_y):
        stats.game_active = True
        downcontrol.dh_uart_send('ProjectStart')
        print("Send Project Start")

    if Grab_button.rect.collidepoint(mouse_x, mouse_y):
        stats.game_active = True
        downcontrol.dh_uart_send('HandDetect')
        print("Send Hand Detect")

    if End_button.rect.collidepoint(mouse_x, mouse_y):
        stats.game_active = True
        downcontrol.dh_uart_send('ProjectEnd')
        print("Send Project End")

    if Link_button.rect.collidepoint(mouse_x, mouse_y):
        stats.game_active = True
        downcontrol.dh_uart_send('LinkCheck')
        print("Send Back Point")

    if Sample_button.rect.collidepoint(mouse_x, mouse_y):
        stats.game_active = True
        downcontrol.dh_uart_send('SampleTrial')
        print("Send Sample Trial")


def check_events(ai_settings, screen, stats, play_button, Grab_button, End_button, Link_button, Sample_button, ship):
    """响应按键和鼠标事件"""
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            print("关闭窗口")
            downcontrol.uart_close() # 关闭串口
            pygame.quit()
            sys.exit()
        elif event.type == pygame.MOUSEBUTTONDOWN:
            mouse_x, mouse_y = pygame.mouse.get_pos()
            check_play_button(stats, play_button, Grab_button, End_button, Link_button, Sample_button, mouse_x, mouse_y)

class GameStats():
    """跟踪游戏的统计信息"""
     
    def __init__(self, ai_settings):
         """初始化统计信息"""
         self.ai_settings =ai_settings
         self.reset_stats()
    
    def reset_stats(self):
        """初始化在游戏运行期间可能变化的统计信息"""
        self.ship_left = self.ai_settings.ship_limit

def run_game():

    # 大寰串口初始化
    downcontrol.uart_init() 

    # 傲博初始化
    robotcontrol.dh_robot_init()

    """初始化游戏并创建一个屏幕对象"""
    pygame.init()
    ai_settings = settings()
    screen = pygame.display.set_mode((ai_settings.screen_width, ai_settings.screen_height))
    pygame.display.set_caption("Electronic Skin")

    # 创建项目开始按钮
    play_button = Button(ai_settings, screen, "Project Start")

    # 创建项目结束按钮
    End_button = EndButton(ai_settings, screen, "Project End")

    # 创建抓取开始按钮
    Grab_button = GrabButton(ai_settings, screen, "Hand Detect")

    # 创建链路检测按钮
    Link_button = LinkButton(ai_settings, screen, "Back Point")

    # 创建样品抓取按钮
    Sample_button = SampleButton(ai_settings, screen, "Sample Trial")   
    
    # 创建一个用于存储游戏统计信息的实例
    stats = GameStats(ai_settings)

    # 创建公司logo
    ship = imageslogo(screen)

    print("init end")

    # 开始游戏的主循环
    while True:
        # 监视键盘和鼠标事件
        check_events(ai_settings, screen, stats, play_button, Grab_button, End_button, Link_button, Sample_button, ship)

        # 更新显示
        update_screen(ai_settings, screen, ship, play_button, Grab_button, End_button, Link_button, Sample_button)

        # 接收数据
        downcontrol.dh_uart_recv() 

        # 解析串口数据
        downcontrol.dh_uart_listdata_analysis() # 矿泉水瓶中倒水场景 分布操作
        # downcontrol.dh_uart_listdata_analysis_one()  #抓矿泉水平,倒水改成晃动矿泉水瓶，任务开始一键完成

if __name__ == '__main__':
    run_game()
      