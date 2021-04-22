from quad_splitter import QuadSplitter
import argparse
import cv2
import os

OUT_DIR = "quadrants"

parser = argparse.ArgumentParser(description='split images into annotated quadrants')
parser.add_argument('dir', type=str,
                    help='directory containing images')
parser.add_argument('color', type=str,
                    help='annotation color of quadrants')

args = parser.parse_args()


class ImgsToAnnots:
    def __init__(self):
        invalid = False
        self.annots = ""
        # initialize center detector, and create output directory
        self.quad_splitter = QuadSplitter()
        out_dir = os.path.join(args.dir, OUT_DIR)
        os.mkdir(out_dir)

        # loop through images, split them, save each quadrant with its annotation
        for image_name in os.listdir(args.dir):
            if image_name == OUT_DIR:
                continue
            img_path = os.path.join(args.dir, image_name)
            quads = self.quad_splitter.get_quads(img_path)  # return the x,y location of the image center

            for i in range(4):
                new_name = f"{image_name.split('.')[0]}_{i}.jpg"
                out_path = os.path.join(out_dir, new_name)
                try:
                    cv2.imwrite(out_path, quads[i])
                except Exception as e:
                    invalid = True
                    break
                self.annots += ",".join([new_name, args.color]) + "\n"
            if invalid:
                continue

        annots_path = os.path.join(out_dir, "annot.csv")
        with open(annots_path, "w") as out:
            out.write(self.annots)


if __name__ == "__main__":