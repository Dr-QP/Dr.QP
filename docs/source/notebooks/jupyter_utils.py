import IPython
from pygments import highlight
from pygments.formatters import HtmlFormatter
from pygments.lexers import PythonLexer


def display_file(
    file_path, start_line=-1, end_line=-1, start_after=None, end_before=None, style='default'
):
    with open(file_path) as f:
        code = f.readlines()
        if start_line > 0:
            code = code[start_line:end_line]
        code_str = ''.join(code)

        if start_after:
            end_index = -1
            if end_before:
                end_index = code_str.index(end_before)

            code_str = code_str[code_str.index(start_after) + len(start_after) : end_index]

    formatter = HtmlFormatter(style=style)
    return IPython.display.HTML(
        f'<style type="text/css">{formatter.get_style_defs()}</style>'
        # + f'<h3>"{file_path}", lines {start_line} to {end_line}</h3>'
        + highlight(code_str, PythonLexer(), formatter)
    )


# # style = 'default' # is good for light theme
# style = 'monokai' # good for dark theme

# html = display_file('gait_generators.py', 20, 40, style=style)
# display(html)
