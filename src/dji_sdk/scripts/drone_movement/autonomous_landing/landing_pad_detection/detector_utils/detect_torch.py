import argparse
from sys import platform

import torch.backends.cudnn as cudnn
from src.models import *  # set ONNX_EXPORT in models.py

from src.utils.datasets import *
from src.utils.utils import *
# detectNet = detector(weights, config, conf_thresh=conf_thresh, netsize=cfg_size, nms_thresh=nms_thresh, gpu=True, classes_file=classes_file)
# detectNet = detector_torch((weights, 640, conf_thresh=conf_thresh, nms_thresh=nms_thresh)

class detector_torch():
    def __init__(self,weights=None,config = None,classes = None,imgsz=640,augment=False,conf_thresh=0.5,nms_thresh=0.4,device='',agnostic_nms = False):
        self.weights = weights
        self.conf_thresh = conf_thresh
        self.nms_thresh=nms_thresh
        self.source=None
        self.config = config
        self.augment=augment
        self.agnostic_nms = agnostic_nms
        # Initialize
        self.device = torch_utils.select_device(device='cpu' if device =='' else str(device))
        self.half = self.device.type != 'cpu'  # half precision only supported on CUDA
        self.imgsz = imgsz
        # Initialize model
        self.model = Darknet( self.config, self.imgsz)
        if weights.endswith('.pt'):  # pytorch format

            self.model.load_state_dict(torch.load(weights, map_location=self.device)['model'])
        else:  # darknet format
            load_darknet_weights(self.model, weights)

            # Get names and colors
        self.classes  = load_classes(classes)

        # Eval mode
        self.model.to(self.device).eval()


        if self.half:
            self.model.half()  # to FP16

    def detect(self,source):
        # self.webcam = source == '0' or source.startswith('rtsp') or source.startswith('http') or source.endswith('.txt')
        self.source = source
        # Set Dataloader
        vid_path, vid_writer = None, None
        # if self.webcam:
        cudnn.benchmark = True  # set True to speed up constant image size inference
        dataset = Loadimg(self.source , img_size=self.imgsz)
        # else:
        # dataset = LoadImages(self.source , img_size=self.imgsz)

            # Run inference
        # t0 = time.time()
        img = torch.zeros((1, 3, self.imgsz, self.imgsz), device=self.device)  # init img
        _ = self.model(img.half() if self.half else img) if self.device.type != 'cpu' else None  # run once
        img,img0= dataset
        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Inference
        # t1 = torch_utils.time_synchronized()
        pred = self.model(img, augment=self.augment)[0]

        # Apply NMS
        pred = non_max_suppression(pred, self.conf_thresh, self.nms_thresh,
                                   merge=False,  agnostic=self.agnostic_nms)
        # t2 = torch_utils.time_synchronized()
        detections = []
        ix=[]
        boxes=[]
        confs = []
        classes = []
        count=0
        if len(pred)>0:
            for i, det in enumerate(pred):
                if det is not None:

                    det[:, :4] = scale_coords(img.shape[2:], det[:, :4], img0.shape).round()

                    for *xyxy, conf, cls in det:
                        xywh = xyxy2xywh(xyxy)
                        ix.append([count])
                        boxes.append(xywh)
                        confs.append(float(conf))
                        classes.append(int(cls))
                        count+=1
                        # detections.append((i, xywh, float(conf), int(cls)))
                        # print(i, xywh, float(conf), int(cls))
                # else:
                #     print("none detections")
        return ix,boxes,classes,confs

def xyxy2xywh(x):
    x1,y1,x2,y2 = int(x[0]),int(x[1]),int(x[2]),int(x[3])
    x,y,w,h = x1,y1, x2-x1,y2-y1
    return [x,y,w,h]
#
# def detect(save_img=False):
#     out, source, weights, view_img, save_txt, imgsz = \
#         opt.output, opt.source, opt.weights, opt.view_img, opt.save_txt, opt.img_size
#     webcam = source == '0' or source.startswith('rtsp') or source.startswith('http') or source.endswith('.txt')
#
#
#     # Set Dataloader
#     vid_path, vid_writer = None, None
#     if webcam:
#         view_img = True
#         cudnn.benchmark = True  # set True to speed up constant image size inference
#         dataset = LoadStreams(source, img_size=imgsz)
#     else:
#         save_img = True
#         dataset = LoadImages(source, img_size=imgsz)
#
#     # Get names and colors
#     names = model.module.names if hasattr(model, 'module') else model.names
#     colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(names))]
#
#     # Run inference
#     t0 = time.time()
#     img = torch.zeros((1, 3, imgsz, imgsz), device=device)  # init img
#     _ = model(img.half() if half else img) if device.type != 'cpu' else None  # run once
#     for path, img, im0s, vid_cap in dataset:
#         img = torch.from_numpy(img).to(device)
#         img = img.half() if half else img.float()  # uint8 to fp16/32
#         img /= 255.0  # 0 - 255 to 0.0 - 1.0
#         if img.ndimension() == 3:
#             img = img.unsqueeze(0)
#
#         # Inference
#         t1 = torch_utils.time_synchronized()
#         pred = model(img, augment=opt.augment)[0]
#
#         # Apply NMS
#         pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres, classes=opt.classes, agnostic=opt.agnostic_nms)
#         t2 = torch_utils.time_synchronized()
#
#         # Apply Classifier
#         if classify:
#             pred = apply_classifier(pred, modelc, img, im0s)
#
#         # Process detections
#         for i, det in enumerate(pred):  # detections per image
#             if webcam:  # batch_size >= 1
#                 p, s, im0 = path[i], '%g: ' % i, im0s[i].copy()
#             else:
#                 p, s, im0 = path, '', im0s
#
#             save_path = str(Path(out) / Path(p).name)
#             txt_path = str(Path(out) / Path(p).stem) + ('_%g' % dataset.frame if dataset.mode == 'video' else '')
#             s += '%gx%g ' % img.shape[2:]  # print string
#             gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
#             if det is not None and len(det):
#                 # Rescale boxes from img_size to im0 size
#                 det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()
#
#                 # Print results
#                 for c in det[:, -1].unique():
#                     n = (det[:, -1] == c).sum()  # detections per class
#                     s += '%g %ss, ' % (n, names[int(c)])  # add to string
#
#                 # Write results
#                 for *xyxy, conf, cls in det:
#                     if save_txt:  # Write to file
#                         xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
#                         with open(txt_path + '.txt', 'a') as f:
#                             f.write(('%g ' * 5 + '\n') % (cls, *xywh))  # label format
#
#                     if save_img or view_img:  # Add bbox to image
#                         label = '%s %.2f' % (names[int(cls)], conf)
#                         plot_one_box(xyxy, im0, label=label, color=colors[int(cls)], line_thickness=3)
#
#             # Print time (inference + NMS)
#             print('%sDone. (%.3fs)' % (s, t2 - t1))
#
#             # Stream results
#             if view_img:
#                 cv2.imshow(p, cv2.resize(im0,(1280,720)))
#                 if cv2.waitKey(1) == ord('q'):  # q to quit
#                     raise StopIteration
#
#             # Save results (image with detections)
#             if save_img:
#                 if dataset.mode == 'images':
#                     cv2.imwrite(save_path, im0)
#                 else:
#                     if vid_path != save_path:  # new video
#                         vid_path = save_path
#                         if isinstance(vid_writer, cv2.VideoWriter):
#                             vid_writer.release()  # release previous video writer
#
#                         fourcc = 'mp4v'  # output video codec
#                         fps = vid_cap.get(cv2.CAP_PROP_FPS)
#                         w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
#                         h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
#                         vid_writer = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*fourcc), fps, (w, h))
#                     vid_writer.write(im0)
#
#     if save_txt or save_img:
#         print('Results saved to %s' % os.getcwd() + os.sep + out)
#         if platform == 'darwin' and not opt.update:  # MacOS
#             os.system('open ' + save_path)
#
#     print('Done. (%.3fs)' % (time.time() - t0))
#
#
# if __name__ == '__main__':
#     parser = argparse.ArgumentParser()
#     parser.add_argument('--weights', nargs='+', type=str, default='yolov5s.pt', help='model.pt path(s)')
#     parser.add_argument('--source', type=str, default='inference/images', help='source')  # file/folder, 0 for webcam
#     parser.add_argument('--output', type=str, default='inference/output', help='output folder')  # output folder
#     parser.add_argument('--img-size', type=int, default=640, help='inference size (pixels)')
#     parser.add_argument('--conf-thres', type=float, default=0.4, help='object confidence threshold')
#     parser.add_argument('--iou-thres', type=float, default=0.5, help='IOU threshold for NMS')
#     parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
#     parser.add_argument('--view-img', action='store_true', help='display results')
#     parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
#     parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --class 0, or --class 0 2 3')
#     parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
#     parser.add_argument('--augment', action='store_true', help='augmented inference')
#     parser.add_argument('--update', action='store_true', help='update all models')
#     opt = parser.parse_args()
#     print(opt)
#
#     with torch.no_grad():
#         if opt.update:  # update all models (to fix SourceChangeWarning)
#             for opt.weights in ['yolov5s.pt', 'yolov5m.pt', 'yolov5l.pt', 'yolov5x.pt', 'yolov3-spp.pt']:
#                 detect()
#                 create_pretrained(opt.weights, opt.weights)
#         else:
#             detect()
