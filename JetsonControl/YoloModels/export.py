import torch
import argparse
from models.yolo import Model  # din repo YOLOv10

def export_onnx(weights_path, output_path='yolov10_export.onnx', img_size=640, batch_size=1):
    # Încarcă modelul
    ckpt = torch.load(weights_path, map_location='cpu')
    model = Model(ckpt['model'].yaml)
    model.load_state_dict(ckpt['model'].state_dict())
    model.eval()

    # Dummy input
    dummy_input = torch.randn(batch_size, 3, img_size, img_size)

    # Export ONNX
    torch.onnx.export(
        model,
        dummy_input,
        output_path,
        input_names=['images'],
        output_names=['output'],
        opset_version=12,
        verbose=False
    )
    print(f"✅ Exportat cu succes: {output_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', type=str, required=True, help='Calea către fișierul .pt')
    parser.add_argument('--output', type=str, default='yolov10_export.onnx', help='Nume fișier ONNX')
    parser.add_argument('--img', type=int, default=640, help='Dimensiune input')
    parser.add_argument('--batch', type=int, default=1, help='Batch size')
    args = parser.parse_args()

    export_onnx(args.weights, args.output, args.img, args.batch)
