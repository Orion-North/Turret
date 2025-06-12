import cv2
from ultralytics import YOLO
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk

model = YOLO("yolov8n.pt")

class DefenseDashboard:
    def __init__(self, root):
        self.root = root
        root.title("AUTONOMOUS TURRET DEMO")
        root.configure(bg='black')

        # ---- STYLE ----
        style = ttk.Style(root)
        style.theme_use('default')
        style.configure('TLabel',      background='black', foreground='lime')
        style.configure('Treeview',    background='black', foreground='lime', fieldbackground='black')
        style.configure('Treeview.Heading', background='black', foreground='lime')

        # ---- TITLE ----
        title = tk.Label(root,
                         text="AUTONOMOUS TURRET DEMO",
                         font=("Consolas", 18, "bold"),
                         bg="black",
                         fg="lime")
        title.grid(row=0, column=0, columnspan=2, pady=(10, 5))

        # ---- CAMERA SELECT ----
        cam_label = tk.Label(root, text="Camera:", font=("Consolas", 12), bg="black", fg="lime")
        cam_label.grid(row=1, column=0, sticky="e", padx=(10, 0))

        # detect cams 0–5
        cams = []
        for i in range(6):
            cap = cv2.VideoCapture(i)
            if cap.read()[0]:
                cams.append(i)
                cap.release()

        self.cam_var = tk.IntVar(value=cams[0] if cams else 0)
        self.cam_dropdown = ttk.Combobox(root,
                                         values=cams,
                                         textvariable=self.cam_var,
                                         state="readonly",
                                         width=5,
                                         font=("Consolas", 12))
        self.cam_dropdown.grid(row=1, column=1, sticky="w", padx=(0, 10))
        self.cam_dropdown.bind("<<ComboboxSelected>>", self._change_cam)

        # ---- VIDEO CANVAS ----
        self.canvas = tk.Canvas(root,
                                width=640,
                                height=480,
                                bg='black',
                                highlightthickness=2,
                                highlightbackground='lime')
        self.canvas.grid(row=2, column=0, padx=10, pady=10)

        # ---- COORDINATE TABLE ----
        cols = ("ID", "X", "Y", "W", "H")
        self.tree = ttk.Treeview(root, columns=cols, show="headings", height=20)
        for c in cols:
            self.tree.heading(c, text=c)
            self.tree.column(c, anchor="center", width=80)
        self.tree.grid(row=2, column=1, padx=10, pady=10, sticky="n")

        # open initial camera
        self.cap = None
        self._open_cam(self.cam_var.get())
        self._update()

    def _open_cam(self, idx):
        if self.cap:
            self.cap.release()
        self.cap = cv2.VideoCapture(idx)

    def _change_cam(self, event=None):
        self._open_cam(self.cam_var.get())

    def _update(self):
        ret, frame = (self.cap.read() if self.cap else (False, None))
        people = []

        if ret:
            # detect
            results = model(frame, verbose=False)[0]
            if hasattr(results, "boxes"):
                for i, (box, cls, conf) in enumerate(zip(results.boxes.xyxy,
                                                         results.boxes.cls,
                                                         results.boxes.conf), start=1):
                    if int(cls) == 0 and conf >= 0.5:
                        x1, y1, x2, y2 = map(int, box)
                        w, h = x2 - x1, y2 - y1
                        label = f"T{i}"
                        people.append((label, x1, y1, w, h))
                        # draw box & label
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(frame,
                                    label,
                                    (x1, y1 - 8),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.6,
                                    (0, 255, 0),
                                    2)

            # show frame
            img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            imgtk = ImageTk.PhotoImage(image=Image.fromarray(img))
            self.canvas.imgtk = imgtk
            self.canvas.create_image(0, 0, anchor="nw", image=imgtk)

            # update table
            for item in self.tree.get_children():
                self.tree.delete(item)
            for (lid, x, y, w, h) in people:
                self.tree.insert("", "end", values=(lid, x, y, w, h))

        # schedule next frame
        self.root.after(30, self._update)


if __name__ == "__main__":
    root = tk.Tk()
    app = DefenseDashboard(root)
    root.mainloop()
