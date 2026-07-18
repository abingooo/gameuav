import os

# 获取项目根目录
def get_project_root():
    """
    获取项目的根目录路径
    
    Returns:
        str: 项目根目录的绝对路径
    """
    return os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

# 加载提示词模板
def load_prompt_template(template_name):
    """
    从prompt文件夹加载提示词模板
    
    Args:
        template_name: 模板文件名（不含路径）
        
    Returns:
        str: 模板字符串
    """
    prompt_dir = os.path.join(get_project_root(), 'prompt')
    file_path = os.path.join(prompt_dir, template_name)
    
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"提示词模板文件不存在: {file_path}")
    
    with open(file_path, 'r', encoding='utf-8') as f:
        return f.read()

# 使用变量填充模板
def format_prompt(template_name, **kwargs):
    """
    加载模板并使用变量填充
    
    Args:
        template_name: 模板文件名
        **kwargs: 用于填充模板的变量
        
    Returns:
        str: 填充后的完整提示词
    """
    template = load_prompt_template(template_name)
    return template.format(**kwargs)