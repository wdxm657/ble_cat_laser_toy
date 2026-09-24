import os

import qrcode
from PIL import Image

# 输出文件夹与图片尺寸
OUTPUT_DIR = "qrcode_images"
SIZE = 300  # 像素：300 x 300


def generate_qrcodes():
    # 创建输出目录（已存在则忽略）
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    for i in range(100):
        text = str(i)  # 内容为 "0" ~ "99"

        # 构建二维码对象
        qr = qrcode.QRCode(
            version=None,                                   # 自动选择合适的版本
            error_correction=qrcode.constants.ERROR_CORRECT_M,  # 纠错等级 M
            box_size=10,                                    # 每个模块的像素数（会被后续缩放覆盖）
            border=4,                                       # 四周留白（4 个模块宽）
        )
        qr.add_data(text)
        qr.make(fit=True)

        # 生成 PIL 图像
        img = qr.make_image(fill_color="black", back_color="white")

        # 取到真正的 PIL.Image 对象，并缩放到 300x300
        # 使用 NEAREST 最近邻缩放，保证二维码黑白模块边缘锐利、不模糊
        pil_img = img.get_image()
        pil_img = pil_img.resize((SIZE, SIZE), Image.NEAREST)

        # 保存
        file_path = os.path.join(OUTPUT_DIR, f"{text}.png")
        pil_img.save(file_path)
        print(f"已生成: {file_path}")

    print(f"\n完成！共生成 100 张二维码，保存在 '{OUTPUT_DIR}' 文件夹中。")


if __name__ == "__main__":
    generate_qrcodes()