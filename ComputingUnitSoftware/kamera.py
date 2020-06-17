import cv2
#import matplotlib.pyplot as plt
from PIL import Image, ImageDraw
import numpy as np
import webcolors
import math
import os
import re
from xml.dom.minidom import parse, getDOMImplementation
from datetime import date, datetime
from collections import Counter
#import threading
import time



def filtruj(zdjecie):

    krawedzie = cv2.Canny(zdjecie, 100, 200, 3, L2gradient=True)
    #plt.imsave('krawedzie.png', krawedzie, cmap='gray', format='png')

    return krawedzie


def znajdz_punkty(zCannyowany, tolerancja):
    stare_biale = np.array(np.where(zCannyowany == 255))

    X = stare_biale[1]
    Y = stare_biale[0]

    biale = stare_biale.copy()
    biale[0] = X
    biale[1] = Y

    tablica_krawedzi = []

    for n in range(len(biale[0])):
        k = tuple(biale[:, n])
        if (k[0] > mnoznikTol * tolerancja and k[0] < zCannyowany.shape[1] - (mnoznikTol * tolerancja) and k[
            1] > tolerancja and k[1] < zCannyowany.shape[0] - tolerancja):
            tablica_krawedzi.append(tuple(biale[:, n]))

    return tablica_krawedzi


def otaczaj_wypukle(listaPkt, zdjecie):
    def centroid(vertexes):
        _x_list = [vertex[0] for vertex in vertexes]
        _y_list = [vertex[1] for vertex in vertexes]
        _len = len(vertexes)
        _x = sum(_x_list) / _len
        _y = sum(_y_list) / _len
        return (int(_x), int(_y))

    def takeSecond(elem):
        return elem[1]

    listaPkt.sort(key=takeSecond, reverse=True)
    srodek = (centroid(listaPkt))

    listaPkt = np.asarray(listaPkt)

    extLeft = tuple(listaPkt[listaPkt[:, 0].argmin()])
    extRight = tuple(listaPkt[listaPkt[:, 0].argmax()])
    extTop = tuple(listaPkt[listaPkt[:, 1].argmin()])
    extBot = tuple(listaPkt[listaPkt[:, 1].argmax()])

    extreme_points = [extLeft, extRight, extTop, extBot, srodek]

    hull = cv2.convexHull(listaPkt, True)

    # cv2.imshow('otoczkowane', zdjecie)
    # cv2.waitKey()

    return hull, extreme_points


def sprawdz_kolor(otoczone, zdjecie):
    zdjecie = cv2.cvtColor(zdjecie, cv2.COLOR_BGR2RGB)
    zdjecie = Image.fromarray(zdjecie)
    zdjecie = zdjecie.convert("RGBA")

    imArray = np.array(zdjecie)

    red = imArray[:, :, 0]
    green = imArray[:, :, 1]
    blue = imArray[:, :, 2]

    maskIm = Image.new('L', (imArray.shape[1], imArray.shape[0]), 0)

    otoczone_tupla = []
    for n in otoczone:
        otoczone_tupla.append(tuple(n[0]))

    ImageDraw.Draw(maskIm).polygon(otoczone_tupla, outline=1, fill=1)
    mask = np.array(maskIm)
    newImArray = np.empty(imArray.shape, dtype='uint8')

    newImArray[:, :, 0] = mask * red
    newImArray[:, :, 1] = mask * green
    newImArray[:, :, 2] = mask * blue
    newImArray[:, :, 3] = mask * 255

    czerwony = newImArray[:, :, 0]
    zielony = newImArray[:, :, 1]
    niebieski = newImArray[:, :, 2]
    warunek = np.where(newImArray[:, :, 3] == 255)

    czerwony = np.array(warunek)
    zielony = np.array(warunek)
    niebieski = np.array(warunek)

    sredni_czerwony = int(sum(newImArray[czerwony[0], czerwony[1], 0]) / len(czerwony[0]))
    sredni_zielony = int(sum(newImArray[zielony[0], zielony[1], 1]) / len(zielony[0]))
    sredni_niebieski = int(sum(newImArray[niebieski[0], niebieski[1], 2]) / len(niebieski[0]))

    kolor = (sredni_czerwony, sredni_zielony, sredni_niebieski)
    kolor_hex = "{:02x}{:02x}{:02x}".format(int(kolor[0]), int(kolor[1]), int(kolor[2]))

    def closest_colour(requested_colour):  
        min_colours = {}
        for key, name in webcolors.css3_hex_to_names.items():
            r_c, g_c, b_c = webcolors.hex_to_rgb(key)
            rd = (r_c - requested_colour[0]) ** 2
            gd = (g_c - requested_colour[1]) ** 2
            bd = (b_c - requested_colour[2]) ** 2
            min_colours[(rd + gd + bd)] = name
        return min_colours[min(min_colours.keys())]

    def get_colour_name(requested_colour):
        try:
            closest_name = actual_name = webcolors.rgb_to_name(requested_colour, spec=webcolors.CSS3)
        except ValueError:
            closest_name = closest_colour(requested_colour)
            actual_name = None
        return actual_name, closest_name

    actual_name, closest_name = get_colour_name(kolor)

    #newIm = Image.fromarray(newImArray, "RGBA")
    #newIm.save("wyciety_obiekt.png")

    return newImArray, kolor, kolor_hex, actual_name, closest_name


def zmierz(h, w):
    # dla kamery w odleglosci ~20cm -> kalibracja[0]
    kalibracja = [[276, 441], [207, 334], [174, 280], [146, 227], [120, 194], [105, 168]]  # 15 20 25 30 35 40

    scaleX = 5.4 / kalibracja[2][0]
    scaleY = 8.5 / kalibracja[2][1]

    width = w * scaleX
    height = h * scaleY

    # print('maksymalna szerokosc: %0.2f cm \n maksymalna wysokosc: %0.2f cm' % (width, height))

    return height, width


def wytnij_roi(otoczone, ekstremalne, zdjecie_krawedzie):
    # cv2.imshow('ZDJECIEKRAWEIDZE', zdjecie_krawedzie)
    # cv2.waitKey()

    otoczone_tupla = []
    for n in otoczone:
        otoczone_tupla.append(tuple(n[0]))

    otoczone_tupla = np.asarray(otoczone_tupla)

    nowy = Image.new('L', (zdjecie_krawedzie.shape[1], zdjecie_krawedzie.shape[0]), 255)
    nowy = np.array(nowy)

    cv2.fillConvexPoly(nowy, otoczone_tupla, [0, 0, 0], lineType=8, shift=0)  # robie czarno-bialy

    # cv2.imshow('zmniejszone', nowy)
    # cv2.waitKey()

    lewyGora = tuple(map(min, zip(*otoczone_tupla)))

    h = abs(ekstremalne[3][1] - ekstremalne[2][1])
    w = abs(ekstremalne[1][0] - ekstremalne[0][0])

    ROI = nowy[(lewyGora[1] - 15):(lewyGora[1] + h + 15), (lewyGora[0] - 15):(lewyGora[0] + w + 15)]
    zmniejszone = cv2.resize(ROI, (120, 160))

    # cv2.imshow('zmniejszone', zmniejszone)
    # cv2.imwrite("male_ROI.png", zmniejszone)

    # cv2.waitKey()

    return zmniejszone, h, w


def sprawdz_ksztalt(roi, h, w):
    def mean2(x):
        y = np.sum(x) / np.size(x)
        return y

    def corr2(a, b):
        a = a - mean2(a)
        b = b - mean2(b)

        r = (a * b).sum() / math.sqrt((a * a).sum() * (b * b).sum())
        return r

    korelacja = []
    ksztalty = []
    path = "obrazy/"
    for n in (os.listdir(path)):
        ima2 = cv2.imread(path + "/" + n)
        ima2 = cv2.cvtColor(ima2, cv2.COLOR_BGR2GRAY)
        ima2 = cv2.bitwise_not(ima2)

        korelacja.append(corr2(roi, ima2))
        ksztalty.append(n)

    korelacja = np.asarray(korelacja)
    # print(korelacja)

    ksztalt = (ksztalty[int(np.argmax(korelacja))])
    ksztalt = re.sub('\.png$', '', ksztalt)
    ksztalt = re.sub("\d+", '', ksztalt)

    if ksztalt == 'Prostokat' and (
            (h > 0.95 * w and h < 1.05 * w) or (w > 0.95 * h and w < 1.05 * h)): ksztalt = 'Kwadrat'

    return ksztalt

def stworzXML():
    impl = getDOMImplementation()
    newdoc = impl.createDocument(None, str(date.today()), None)
    top_element = newdoc.documentElement
    return newdoc, top_element


def uzupelnijXML(newdoc, top_element, kolor, hex, nazwa, najblizsza_nazwa, ksztalt, wysokosc, szerokosc):
    obiektNode = newdoc.createElement("Sortowanie ")

    kolorNode = newdoc.createElement("Kolor(RGB)")
    kolorText = newdoc.createTextNode(str(kolor))
    kolorNode.appendChild(kolorText)

    kolorHexNode = newdoc.createElement("Kolor(hex)")
    kolorHexText = newdoc.createTextNode('0x' + str(hex))
    kolorHexNode.appendChild(kolorHexText)

    kolornazwaNode = newdoc.createElement("Kolor(nazwa)")
    kolornazwaText = newdoc.createTextNode(str(nazwa) if nazwa != None else str(najblizsza_nazwa))
    kolornazwaNode.appendChild(kolornazwaText)

    ksztaltNode = newdoc.createElement("Ksztalt")
    ksztaltText = newdoc.createTextNode(ksztalt)
    ksztaltNode.appendChild(ksztaltText)

    hNode = newdoc.createElement("MaxWysokosc(cm)")
    hText = newdoc.createTextNode(str('%0.2f' % wysokosc))
    hNode.appendChild(hText)

    wNode = newdoc.createElement("MaxSzerokosc(cm)")
    wText = newdoc.createTextNode(str('%0.2f' % szerokosc))
    wNode.appendChild(wText)

    top_element.appendChild(obiektNode)
    obiektNode.appendChild(kolorNode)
    obiektNode.appendChild(kolorHexNode)
    obiektNode.appendChild(kolornazwaNode)
    obiektNode.appendChild(ksztaltNode)
    obiektNode.appendChild(hNode)
    obiektNode.appendChild(wNode)

    print(newdoc.toprettyxml())
    newdoc.writexml(open('skanowania.xml', 'w'), indent="  ", addindent="  ", newl='\n')


def cyk_foto():
    os.system("fswebcam -c ~/.fswebcam.conf")
    image = cv2.imread("/home/pi/zdj.jpg")
    #cv2.imwrite("zdjecie_kamera.png", image)

    return image


def klasyfikuj(kol=False, ksz=False):
    
    tolerancja = 170

    kolory = []
    ksztalty = []
    cnt = 0
    limit_down = 5
    limit_up = 10

    sklasyfikowanoKsztalt = kol
    sklasyfikowanoKolor = ksz

    while cnt < limit_up:

        frame = cyk_foto()
        zdjecie_krawedzie = filtruj(frame)
        punkty = znajdz_punkty(zdjecie_krawedzie, tolerancja)

        pov = np.array([[[mnoznikTol * tolerancja, tolerancja],
                         [mnoznikTol * tolerancja, frame.shape[0] - tolerancja],
                         [frame.shape[1] - mnoznikTol * tolerancja, frame.shape[0] - tolerancja],
                         [frame.shape[1] - mnoznikTol * tolerancja, tolerancja]]], np.int32)
        #cv2.polylines(frame, [pov], True, (255, 00, 00), 1)

        if (len(punkty) > 1):
            cnt += 1
            otoczka, skrajne = otaczaj_wypukle(punkty, frame)
            #cv2.polylines(frame, otoczka, True, (255, 255, 00), 5)

            _, _, _, _, najblizsza_nazwa = sprawdz_kolor(otoczka, frame)
            roi, h, w = wytnij_roi(otoczka, skrajne, zdjecie_krawedzie)  # zwraca zmiejszone roi
            wysokosc, szerokosc = zmierz(h, w)
            ksztalt = sprawdz_ksztalt(roi, wysokosc, szerokosc)
			#uzupelnijXML(n, t, kolor, kolor_hex, nazwa, najblizsza_nazwa, ksztalt, wysokosc, szerokosc)
			
            kolory.append(najblizsza_nazwa)
            ksztalty.append(ksztalt)

        #cv2.imshow('frame', frame)
        #cv2.waitKey(1)

        if cnt >= limit_down:
            if Counter(ksztalty).most_common(1)[0][1] / len(ksztalty) * 100 >= 70:
                sklasyfikowanoKsztalt = True
            if Counter(kolory).most_common(1)[0][1] / len(kolory) * 100 >= 70:
                sklasyfikowanoKolor = True

        if sklasyfikowanoKsztalt and sklasyfikowanoKolor:
            break

    #print(ksztalty)
    #print(kolory)

    ksztaltName = Counter(ksztalty).most_common(1)[0][0]
    ksztaltPercent = Counter(ksztalty).most_common(1)[0][1] / len(ksztalty) * 100

    kolorName = Counter(kolory).most_common(1)[0][0]
    kolorPercent = Counter(kolory).most_common(1)[0][1] / len(kolory) * 100

    #cap.release()
    #cv2.destroyAllWindows()

    return sklasyfikowanoKsztalt, ksztaltName, ksztaltPercent, sklasyfikowanoKolor, kolorName, kolorPercent


mnoznikTol = 1.15
if __name__ == "__main__":
    a = time.time()
    print(klasyfikuj())
    b = time.time()
    print(b - a)

