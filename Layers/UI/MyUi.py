import tkinter

class MyUi:
    def __init__(self):
        self.root = tkinter.Tk()
        self.label = tkinter.Label(self.root, text="Hello, world!")
        self.label.pack()
        self.root.mainloop()

     