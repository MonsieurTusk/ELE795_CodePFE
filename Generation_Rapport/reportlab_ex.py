from reportlab.platypus import SimpleDocTemplate
from reportlab.lib.styles import getSampleStyleSheet
from reportlab.platypus import Paragraph
from reportlab.platypus import PageBreak
from reportlab.platypus import Image

#Mise en page
fileName = "ex_rapport.pdf"
documentTitle = "Rapport PFE"
title = "Rapport PFE"
img_path = "/home/lemg/Desktop/HumbleHawksbill.png"

# Creation document
pdf = SimpleDocTemplate(fileName)

# Argument for build
story = []
style = getSampleStyleSheet()

# Text
title = Paragraph("Rapport PFE", style["Heading1"])
p1 = Paragraph("Ce rapport presente les variables suivantes", style["BodyText"])
p2 = Paragraph("Zmin, Zmax, Zmoy, Zcount", style["BodyText"])
p3 = Paragraph("Dmin, Dmax, Dmoy, Dcount", style["BodyText"])
p4 = Paragraph("Une visualisation Rviz", style["BodyText"])
img = open(img_path, "rb")

story.append(title)
story.append(p1)
story.append(p2)
story.append(p3)
story.append(p4)
story.append(Image(img))

# Build
pdf.build(story)


