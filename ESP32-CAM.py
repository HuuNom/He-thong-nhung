from flask import Flask, render_template, Response
import os, sys, cv2, torch, numpy as np, easyocr, time, re, threading
from pathlib import Path
from difflib import SequenceMatcher

app = Flask(__name__)

# --- 1. CẤU HÌNH YOLOv5 ---
BASE_DIR = Path(__file__).resolve().parent
YOLO_ROOT = os.path.join(BASE_DIR, "yolov5")
if YOLO_ROOT not in sys.path:
    sys.path.append(YOLO_ROOT)

from models.common import DetectMultiBackend
from utils.general import non_max_suppression, scale_boxes
from utils.augmentations import letterbox

device = torch.device('cpu')
model = DetectMultiBackend(os.path.join(BASE_DIR, 'yolov5n.pt'), device=device)
stride, names, pt = model.stride, model.names, model.pt

# Chỉ cho phép đọc Chữ và Số để tăng tốc và giảm sai sót
reader = easyocr.Reader(['en'], gpu=False)
ALLOW_CHARS = '0123456789ABCDEFGHKLMNPSTUVXYZ'

history_plates = {}
last_data = {"plate": "", "box": None, "time": 0}
data_lock = threading.Lock()
is_ocr_busy = False 

def is_similar(p1, p2):
    return SequenceMatcher(None, p1, p2).ratio() > 0.8

def process_ocr(crop_img, box_coords):
    """Hàm xử lý OCR đã được tối ưu cho chữ vẽ tay và ảnh nhiễu"""
    global last_data, is_ocr_busy
    is_ocr_busy = True
    try:
        # Tiền xử lý cứu nét: Chuyển xám và phóng to
        gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
        upscale = cv2.resize(gray, None, fx=3, fy=3, interpolation=cv2.INTER_LANCZOS4)
        
        # Đọc OCR
        res = reader.readtext(upscale, detail=0, allowlist=ALLOW_CHARS)
        raw = "".join(res).upper()
        
        # Loại bỏ dấu gạch ngang, dấu chấm, khoảng cách trước khi kiểm tra Regex
        clean_raw = re.sub(r'[^A-Z0-9]', '', raw)
        
        print(f"🔍 AI Đang thử đọc: {raw} -> Sau khi lọc rác: {clean_raw}")

        match = re.search(r'(\d{2})([A-Z])(\d{4,6})', clean_raw)
        
        if match:
            tinh, chu, so = match.group(1), match.group(2), match.group(3)
            # Định dạng hiển thị cho đẹp
            formatted = f"{tinh}{chu}-{so[:3]}.{so[3:]}" if len(so) == 5 else f"{tinh}{chu}-{so}"
            
            now = time.time()
            with data_lock:
                is_duplicate = any(is_similar(formatted, p) for p, t in history_plates.items() if now - t < 30)
                if not is_duplicate:
                    cv2.imwrite(f"oto_captures/{clean_raw}_{int(now)}.jpg", crop_img)
                    history_plates[formatted] = now
                    print(f"✅ XÁC NHẬN BIỂN SỐ: {formatted}")
                
                last_data = {"plate": formatted, "box": box_coords, "time": now}
    except Exception as e:
        print(f"Lỗi OCR: {e}")
    is_ocr_busy = False

def gen_frames():
    global last_data, is_ocr_busy
    stream_url = "http://192.168.165.69:81/stream"
    cap = cv2.VideoCapture(stream_url)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    while True:
        for _ in range(5): cap.grab()
        ret, frame = cap.retrieve()
        if not ret: break

        img = letterbox(frame, 416, stride=stride, auto=pt)[0]
        img = img.transpose((2, 0, 1))[::-1]
        img = np.ascontiguousarray(img).astype(np.float32) / 255.0
        img = torch.from_numpy(img).unsqueeze(0)

        with torch.no_grad():
            pred = model(img)
        pred = non_max_suppression(pred, 0.35, 0.45) 

        for det in pred:
            if len(det):
                det[:, :4] = scale_boxes(img.shape[2:], det[:, :4], frame.shape).round()
                # Chọn khung hình YOLO tin tưởng nhất
                best_det = det[torch.argmax(det[:, 4])]
                x1, y1, x2, y2 = map(int, best_det[:4])
                
                if not is_ocr_busy:
                    crop = frame[y1:y2, x1:x2].copy()
                    threading.Thread(target=process_ocr, args=(crop, (x1, y1, x2, y2)), daemon=True).start()

        with data_lock:
            if last_data["plate"] and (time.time() - last_data["time"] < 1.5):
                b = last_data["box"]
                cv2.rectangle(frame, (b[0], b[1]), (b[2], b[3]), (0, 255, 0), 2)
                cv2.putText(frame, last_data["plate"], (b[0], b[1] - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        ret, buffer = cv2.imencode('.jpg', frame)
        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/')
def index(): return render_template('index.html')

@app.route('/video_feed')
def video_feed(): return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    if not os.path.exists("oto_captures"): os.makedirs("oto_captures")
    print("🚀 Hệ thống đang chạy... Mở trình duyệt tại http://localhost:5000")
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)