#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
自然语言指令发布器

这个脚本提供命令行交互界面，用于向/lang_cmd话题发布自然语言指令。
支持plan和run两种命令类型，用户可以通过输入数字1或2进行选择，并允许重复输入指令直到选择退出。
添加了clear命令，可以清空命令行并重新显示欢迎信息。
"""

import rospy
from quadrotor_msgs.msg import LangCommand
import sys
import signal
import os

def signal_handler(sig, frame):
    """处理Ctrl+C信号，优雅退出"""
    print("\n程序被用户中断，正在退出...")
    rospy.signal_shutdown("User requested shutdown")
    sys.exit(0)

def publish_language_command(publisher, cmd_type, content):
    """
    发布语言指令到话题
    
    Args:
        publisher: ROS话题发布者
        cmd_type: 命令类型 (plan 或 run)
        content: 命令内容
    """
    msg = LangCommand()
    msg.command_type = cmd_type
    msg.content = content
    publisher.publish(msg)
    rospy.loginfo("已发布指令:")
    rospy.loginfo(f"  类型: {cmd_type}")
    rospy.loginfo(f"  内容: {content}")
    print("指令发布成功！\n")

def clear_terminal():
    """清空命令行终端"""
    # 根据不同操作系统使用不同的清空命令
    os.system('cls' if os.name == 'nt' else 'clear')

def show_welcome_message():
    """显示欢迎信息"""
    print("========================================")
    print("         自然语言指令发布器")
    print("========================================")
    print("此程序用于向无人机发布自然语言指令。")
    print("支持的命令类型：")
    print("  1. plan - 发送规划指令")
    print("  2. run  - 发送执行指令")
    print("输入'clear'清空命令行")
    print("输入'q'或'quit'退出程序")
    print("========================================\n")

def main():
    """主函数，初始化ROS节点并提供命令行交互界面"""
    # 设置信号处理
    signal.signal(signal.SIGINT, signal_handler)
    
    # 初始化ROS节点
    rospy.init_node('lang_command_publisher', anonymous=True)
    
    # 创建话题发布者
    pub = rospy.Publisher('/lang_cmd', LangCommand, queue_size=10)
    
    # 等待节点初始化
    rospy.sleep(1.0)
    
    # 显示欢迎信息
    show_welcome_message()
    
    # 命令类型映射
    cmd_type_map = {
        '1': 'plan',
        '2': 'run'
    }
    
    # run命令预定义选项映射
    run_cmd_map = {
        '1': 'go',
        '2': 'cancel'
    }
    
    while not rospy.is_shutdown():
        try:
            # 获取命令类型
            cmd_choice = input("请选择命令类型 (1=plan/2=run): ").strip().lower()
            
            # 检查是否退出
            if cmd_choice in ['q', 'quit', 'exit']:
                print("感谢使用，再见！")
                break
            
            # 检查是否清空命令行
            if cmd_choice == 'clear':
                clear_terminal()
                show_welcome_message()
                continue
            
            # 通过数字或直接输入获取命令类型
            if cmd_choice in cmd_type_map:
                cmd_type = cmd_type_map[cmd_choice]
            elif cmd_choice in ['plan', 'run']:
                cmd_type = cmd_choice
            else:
                print("错误：无效的命令类型，请输入 '1', '2', 'plan' 或 'run'")
                continue
            
            # 获取命令内容
            if cmd_type == 'run':
                # 提供预定义的run命令选项
                print("可选的run命令:")
                print("  1. go    - 执行规划的路径")
                print("  2. cancel - 取消当前规划")
                print("或者输入自定义命令")
                
                run_choice = input("请选择或输入命令内容: ").strip().lower()
                
                # 通过数字或直接输入获取run命令内容
                if run_choice in run_cmd_map:
                    content = run_cmd_map[run_choice]
                else:
                    content = run_choice
                    # 如果用户输入的不是预定义命令，给予提示
                if content not in ['go', 'cancel']:
                    print(f"警告：'{content}' 不是预定义的run命令")
                    confirm = input("是否继续使用自定义命令？ (y/n): ").strip().lower()
                    if confirm != 'y':
                        continue
                # 检查run命令内容是否为clear
                elif content == 'clear':
                    clear_terminal()
                    show_welcome_message()
                    continue
            else:  # plan命令
                content = input("请输入规划命令内容: ").strip()
            
            # 检查内容是否为空
            if not content:
                print("错误：命令内容不能为空")
                continue
            
            # 发布命令
            publish_language_command(pub, cmd_type, content)
            
        except Exception as e:
            rospy.logerr(f"发布指令时出错: {str(e)}")
            print(f"错误：{str(e)}")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("ROS节点被中断")
    except Exception as e:
        print(f"程序异常退出: {str(e)}")
    finally:
        print("程序已结束")