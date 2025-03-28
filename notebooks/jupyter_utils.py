from pygments import highlight
from pygments.lexers import PythonLexer
from pygments.formatters import HtmlFormatter
import IPython


def display_file(file_path, start_line=0, end_line=-1, style='default'):
    with open(file_path) as f:
        code = f.readlines()[start_line:end_line]
        code_str = ''.join(code)

    formatter = HtmlFormatter(style=style)
    return IPython.display.HTML(
        f'<style type="text/css">{formatter.get_style_defs()}</style>'
        + f'<h3>"{file_path}", lines {start_line} to {end_line}</h3>'
        + highlight(code_str, PythonLexer(), formatter)
    )


# # style = 'default' # is good for light theme
# style = 'monokai' # good for dark theme

# html = display_file('gait_generators.py', 20, 40, style=style)
# display(html)
