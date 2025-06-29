import os
import sys

def import_board_module():
    """
    自动切换导入 Board 或 test_board，并返回 (Board模块, 是否为debug环境)
    """
    try:
        current_file_path = os.path.abspath(__file__)
        current_folder_path = os.path.dirname(current_file_path) #conf_pkg
        parent_folder_path = os.path.dirname(current_folder_path) #hexapod7
        grandparent_folder_path = os.path.dirname(parent_folder_path) #HiwonderSDK
        sys.path.append(grandparent_folder_path)
        import Board  # 实际用到的包
        is_debug_mode = False
        return Board, is_debug_mode
    except ImportError:
        from conf_pkg import test_board as Board  # 测试环境
        is_debug_mode = True
        return Board, is_debug_mode



if __name__ == "__main__":
    Board, is_debug_mode = import_board_module()
