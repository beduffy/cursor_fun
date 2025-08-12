from PIL import Image, ImageDraw, ImageFont
import textwrap
import math


def create_code_snippet_image(code, output_path, font_size=18):
    # Load a monospace font
    try:
        font = ImageFont.truetype("DejaVuSansMono.ttf", font_size)
    except:
        font = ImageFont.load_default()
    
    # Calculate image size based on code lines
    lines = code.split('\n')
    max_line_length = max(len(line) for line in lines)
    line_height = font_size + 2
    img_width = max_line_length * font_size // 2
    img_height = len(lines) * line_height
    
    # Create image with white background
    img = Image.new('RGB', (img_width, img_height), color='white')
    d = ImageDraw.Draw(img)
    
    # Draw code lines
    y = 0
    for line in lines:
        d.text((10, y), line, fill='black', font=font)
        y += line_height
    
    img.save(output_path)

def add_arrows_to_image(image_path, output_path, arrows):
    """Add arrows to an existing image
    Args:
        arrows: List of tuples (start_x, start_y, end_x, end_y, color)
    """
    img = Image.open(image_path)
    draw = ImageDraw.Draw(img)
    
    for arrow in arrows:
        x1, y1, x2, y2, color = arrow
        # Draw main line
        draw.line((x1, y1, x2, y2), fill=color, width=2)
        # Draw arrowhead
        draw.regular_polygon((x2, y2, 8), 3, fill=color, rotation=math.degrees(math.atan2(y2-y1, x2-x1)))
    
    img.save(output_path)

# Create code snippet image
code = """def example():
    print('Hello World')
    return 42"""
create_code_snippet_image(code, "code_snippet.png")

# Add arrows
arrows = [
    (50, 30, 150, 30, 'red'),   # Horizontal arrow
    (50, 60, 50, 100, 'blue')   # Vertical arrow
]
add_arrows_to_image("code_snippet.png", "annotated_code.png", arrows)

# Comment out or remove SVG conversion section if not needed
# import cairosvg
# cairosvg.svg2png(url='input.svg', write_to='output.png')  # Ensure input.svg exists

# Optional: Only include Pygments code highlighting if you need it
# from pygments import highlight
# from pygments.lexers import PythonLexer
# from pygments.formatters import ImageFormatter
# code = "print('Hello World')"
# highlight(code, PythonLexer(), ImageFormatter(), "code_image.png")