#!/usr/bin/env python3
"""Generate the printable ArUco home marker used by home_marker_calibrate.py /
home_marker_localize.py to seed AMCL's initial pose at cold start.

  python3 tools/generate_home_marker.py --output /tmp/home_marker.pdf
  python3 tools/generate_home_marker.py --output /tmp/home_marker.png   # just the raw marker

DICT_4X4_250, ID 100 - same dictionary tools/calibrate_camera_charuco.py's ChArUco board uses,
but an ID well outside that board's own range (an 11x8 ChAruco board uses IDs 0-43), so the two
are never confused if both end up in frame at some point.

The PDF embeds the marker at its exact physical size (HOME_MARKER_SIZE_M, home_marker_common.py)
via PDF points - an absolute unit, not image DPI metadata a printer driver could reinterpret - so
printing it at "Actual size" / "100%" / with "Fit to page" OFF reproduces that size exactly. It
also prints a 10 cm reference ruler on the same page: measuring THAT with a ruler after printing
is a much easier way to catch a printer that silently rescaled anyway than measuring the marker's
own pattern, and confirms the scale factor to correct HOME_MARKER_SIZE_M by if it's off.
"""
import argparse
import pathlib

import cv2
import cv2.aruco as aruco
from PIL import Image, ImageDraw, ImageFont

from home_marker_common import HOME_MARKER_DICT, HOME_MARKER_ID, HOME_MARKER_SIZE_M

MARKER_PIXELS = 1000  # render resolution of the black marker square, not its physical size
BORDER_FRACTION = 1 / 8  # white quiet-zone border, as a fraction of MARKER_PIXELS, each side

DPI = 300
PAGE_SIZES_MM = {"a4": (210, 297), "letter": (215.9, 279.4)}
RULER_LENGTH_CM = 10.0
MARGIN_MM = 15


def mm_to_px(mm, dpi=DPI):
    return round(mm / 25.4 * dpi)


def render_marker_array():
    dictionary = aruco.getPredefinedDictionary(HOME_MARKER_DICT)
    marker = aruco.drawMarker(dictionary, HOME_MARKER_ID, MARKER_PIXELS)
    border = round(MARKER_PIXELS * BORDER_FRACTION)
    # White border so the detector's quiet-zone margin isn't right at the paper edge.
    return cv2.copyMakeBorder(marker, border, border, border, border, cv2.BORDER_CONSTANT, value=255), border


def save_png(output):
    marker, _ = render_marker_array()
    cv2.imwrite(str(output), marker)
    print(f"saved {output} ({marker.shape[1]}x{marker.shape[0]} px) - a plain image, not sized "
          f"for printing. Use a .pdf output for a page sized to print at the right scale.")


def save_pdf(output, page_size):
    marker, border = render_marker_array()
    total_px = marker.shape[0]  # square: MARKER_PIXELS + 2*border
    marker_frac = MARKER_PIXELS / total_px  # how much of the rendered square is the black marker
    marker_img_side_mm = (HOME_MARKER_SIZE_M * 1000) / marker_frac

    page_w_mm, page_h_mm = PAGE_SIZES_MM[page_size]
    page_w_px, page_h_px = mm_to_px(page_w_mm), mm_to_px(page_h_mm)
    page = Image.new("RGB", (page_w_px, page_h_px), "white")
    draw = ImageDraw.Draw(page)

    # Marker, scaled with NEAREST to keep hard black/white edges - no antialiasing fringing to
    # confuse the corner detector or blur the ruler's own tick marks.
    marker_img_side_px = mm_to_px(marker_img_side_mm)
    marker_pil = Image.fromarray(marker).resize((marker_img_side_px, marker_img_side_px), Image.NEAREST)
    marker_x = (page_w_px - marker_img_side_px) // 2
    marker_y = mm_to_px(MARGIN_MM + 10)
    page.paste(marker_pil, (marker_x, marker_y))

    font = ImageFont.load_default()
    title_font = ImageFont.load_default()

    def centered_text(y, text, font=font, fill="black"):
        bbox = draw.textbbox((0, 0), text, font=font)
        draw.text(((page_w_px - (bbox[2] - bbox[0])) // 2, y), text, font=font, fill=fill)

    centered_text(mm_to_px(MARGIN_MM), "XLeRobot home marker - print at Actual Size / 100%, Fit to Page OFF",
                  font=title_font)
    centered_text(mm_to_px(MARGIN_MM) + 20,
                  f"DICT_4X4_250, marker ID {HOME_MARKER_ID} - black square must measure "
                  f"{HOME_MARKER_SIZE_M * 100:.1f} cm per side")

    # 10 cm reference ruler, independent of the marker pattern - easy to check precisely with a
    # ruler, and the simplest way to tell a printer silently rescaled the whole page.
    ruler_y = marker_y + marker_img_side_px + mm_to_px(15)
    ruler_len_px = mm_to_px(RULER_LENGTH_CM * 10)
    ruler_x0 = (page_w_px - ruler_len_px) // 2
    tick_h = mm_to_px(4)
    draw.line([(ruler_x0, ruler_y), (ruler_x0 + ruler_len_px, ruler_y)], fill="black", width=3)
    for cm in range(int(RULER_LENGTH_CM) + 1):
        x = ruler_x0 + mm_to_px(cm * 10)
        draw.line([(x, ruler_y - tick_h), (x, ruler_y + tick_h)], fill="black", width=2)
    centered_text(ruler_y + tick_h + mm_to_px(3),
                  f"reference ruler - should measure exactly {RULER_LENGTH_CM:.0f}.0 cm end to end")

    output.parent.mkdir(parents=True, exist_ok=True)
    page.save(str(output), "PDF", resolution=DPI)
    print(f"saved {output} ({page_size}, {page_w_mm:.0f}x{page_h_mm:.0f} mm)")
    print(f"print at Actual Size / 100% scale (NOT Fit to Page), then verify with a ruler:")
    print(f"  - the marker's black square should measure {HOME_MARKER_SIZE_M * 100:.1f} cm per side")
    print(f"  - the reference line should measure exactly {RULER_LENGTH_CM:.0f}.0 cm")
    print(f"if either is off, your printer rescaled the page - fix the print dialog, don't just "
          f"edit HOME_MARKER_SIZE_M to match a wrong print.")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output", type=pathlib.Path, default=pathlib.Path("/tmp/home_marker.pdf"))
    parser.add_argument("--page-size", choices=sorted(PAGE_SIZES_MM), default="a4")
    args = parser.parse_args()

    if args.output.suffix.lower() == ".pdf":
        save_pdf(args.output, args.page_size)
    else:
        save_png(args.output)


if __name__ == "__main__":
    main()
