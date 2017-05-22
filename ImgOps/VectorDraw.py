import VectorOps.VectorMath as VectorMath
from PIL import Image, ImageDraw
import numpy

def create_vector_img2D(vectors, img_size, color, border = 2):
    vector_bounds = VectorMath.get_flat_bounds_of_vectors(vectors)
    center_bounds = vector_bounds.center()
    
    normalization_vector = numpy.array(img_size)/numpy.array([vector_bounds.width + border, vector_bounds.height + border])

    normalized_vectors = numpy.zeros((vectors.shape[0], 2))#vectors * normalization_vector
    center_bounds *= normalization_vector

    for i in range(0, normalized_vectors.shape[0]):
        normalized_vectors[i] = numpy.array([vectors[i][0], vectors[i][1]]) * normalization_vector

    out_img = Image.new('L', img_size)
    out_image = out_img.load()
    center_vector = numpy.array([out_img.size[0]//2, out_img.size[1]//2])

    draw_pixels = numpy.zeros((vectors.shape[0], 2))

    for i in range(0, normalized_vectors.shape[0]):
        pixel_x = int(center_vector[0] + normalized_vectors[i][0] - center_bounds[0])
        pixel_y = int(center_vector[1] - normalized_vectors[i][1] + center_bounds[1])
        draw_pixels[i] = numpy.array([pixel_x, pixel_y])
    image_draw = ImageDraw.Draw(out_img)
    for i in range(0, draw_pixels.shape[0]):

        image_draw.line((draw_pixels[i][0], draw_pixels[i][1], draw_pixels[(i+1)%(draw_pixels.shape[0])][0], draw_pixels[(i+1)%(draw_pixels.shape[0])][1]), fill = color)

    return out_img
