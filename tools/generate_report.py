"""Build the public technical report PDF from the project documentation."""

from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.platypus import (
    BaseDocTemplate,
    Frame,
    Image,
    ListFlowable,
    ListItem,
    PageTemplate,
    Paragraph,
    Spacer,
)


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "docs" / "technical_report.pdf"
PREVIEW = ROOT / "assets" / "images" / "complex_environment.jpg"


class ReportDocTemplate(BaseDocTemplate):
    def __init__(self, filename: Path):
        super().__init__(
            str(filename),
            pagesize=A4,
            rightMargin=22 * mm,
            leftMargin=22 * mm,
            topMargin=20 * mm,
            bottomMargin=20 * mm,
            title="Autonomous Exploration and Mapping with Pioneer 3-DX",
            author="Tianyu Li",
            subject="Autonomous mobile robot mapping and navigation",
        )
        frame = Frame(
            self.leftMargin,
            self.bottomMargin,
            self.width,
            self.height,
            id="body",
        )
        self.addPageTemplates(PageTemplate(id="report", frames=frame, onPage=self.footer))

    @staticmethod
    def footer(canvas, doc):
        canvas.saveState()
        canvas.setStrokeColor(colors.HexColor("#CBD5E1"))
        canvas.line(doc.leftMargin, 14 * mm, A4[0] - doc.rightMargin, 14 * mm)
        canvas.setFillColor(colors.HexColor("#64748B"))
        canvas.setFont("Helvetica", 8)
        canvas.drawString(doc.leftMargin, 9 * mm, "Autonomous Frontier Explorer")
        canvas.drawRightString(A4[0] - doc.rightMargin, 9 * mm, str(doc.page))
        canvas.restoreState()


styles = getSampleStyleSheet()
styles.add(
    ParagraphStyle(
        "ReportTitle",
        parent=styles["Title"],
        fontName="Helvetica-Bold",
        fontSize=24,
        leading=29,
        textColor=colors.HexColor("#0F172A"),
        alignment=TA_CENTER,
        spaceAfter=8 * mm,
    )
)
styles.add(
    ParagraphStyle(
        "Section",
        parent=styles["Heading1"],
        fontName="Helvetica-Bold",
        fontSize=15,
        leading=19,
        textColor=colors.HexColor("#0F4C5C"),
        spaceBefore=5 * mm,
        spaceAfter=2.5 * mm,
    )
)
styles.add(
    ParagraphStyle(
        "Subsection",
        parent=styles["Heading2"],
        fontName="Helvetica-Bold",
        fontSize=11.5,
        leading=15,
        textColor=colors.HexColor("#334155"),
        spaceBefore=3 * mm,
        spaceAfter=1.5 * mm,
    )
)
styles.add(
    ParagraphStyle(
        "BodyTextClean",
        parent=styles["BodyText"],
        fontName="Helvetica",
        fontSize=9,
        leading=13,
        textColor=colors.HexColor("#1E293B"),
        spaceAfter=2.5 * mm,
    )
)
styles.add(
    ParagraphStyle(
        "Caption",
        parent=styles["BodyText"],
        fontName="Helvetica-Oblique",
        fontSize=8,
        leading=11,
        alignment=TA_CENTER,
        textColor=colors.HexColor("#64748B"),
        spaceAfter=3 * mm,
    )
)


def p(text: str):
    return Paragraph(text, styles["BodyTextClean"])


story = [
    Spacer(1, 18 * mm),
    Paragraph("Autonomous Exploration and Mapping<br/>with Pioneer 3-DX", styles["ReportTitle"]),
    Paragraph("<b>Technical Report</b>", styles["Caption"]),
    Spacer(1, 8 * mm),
    Paragraph("Abstract", styles["Section"]),
    p(
        "This project implements an autonomous control system for the Pioneer 3-DX "
        "robot in Webots. It combines a Bayesian occupancy grid for mapping, "
        "frontier-based exploration for autonomous target selection, and hybrid Bug "
        "navigation for obstacle avoidance. The system maps complex, previously "
        "unknown environments without human intervention or preloaded waypoints and "
        "achieved more than 97% observed map coverage in the included scenario."
    ),
    Paragraph("1. System architecture", styles["Section"]),
    p(
        "The controller uses three cooperating layers: perception converts ultrasonic "
        "range measurements into a probabilistic map; planning selects useful boundaries "
        "between known free space and unexplored space; and control drives toward each "
        "target, follows walls around obstructions, and triggers recovery when motion stalls."
    ),
    p("The implementation was developed and evaluated with Webots R2025a on macOS."),
    Paragraph("2. Mapping strategy", styles["Section"]),
    Paragraph("2.1 Bayesian log-odds updates", styles["Subsection"]),
    p(
        "The occupancy grid stores log-odds rather than binary cell states. An obstacle "
        "observation applies an occupied update equivalent to a probability of 0.75; an "
        "observed clear ray applies a conservative free-space update equivalent to 0.4. "
        "Repeated observations strengthen confidence while contradictory readings can "
        "still correct the map."
    ),
    Paragraph("2.2 Inverse sensor model", styles["Subsection"]),
    p(
        "Wide ultrasonic beams tend to smear wall boundaries. For each cell, the "
        "implementation selects the sensor whose axis is closest to the cell bearing. "
        "Range gating ignores near-field noise, maximum-range echoes, and unreliable "
        "distant observations. A safety buffer prevents fluctuating free-space readings "
        "from erasing established walls."
    ),
    Paragraph("2.3 Runtime optimisation", styles["Subsection"]),
    p(
        "Only cells within three metres of the robot are considered for each mapping "
        "update. Mapping runs every third simulation step and rendering every fifteenth "
        "step. This preserves responsive simulation while maintaining a "
        "15-cell-per-metre grid."
    ),
    Image(str(PREVIEW), width=100 * mm, height=62 * mm, kind="proportional"),
    Paragraph("Figure 1. Pioneer 3-DX in the included complex Webots environment.", styles["Caption"]),
    Paragraph("3. Frontier-based exploration", styles["Section"]),
    p(
        "A frontier is an unknown cell adjacent to confidently free space. Candidates "
        "near arena boundaries are rejected, and the planner favours useful middle-distance "
        "goals between 1.5 and 5 metres. This reduces dithering around nearby sensor "
        "artefacts while continuing to push into unexplored regions. If no preferred "
        "candidate exists, the planner falls back to any valid frontier outside the "
        "robot's immediate footprint."
    ),
    p(
        "The high-level controller begins with a full in-place scan, then alternates "
        "between frontier selection and goal-directed exploration. It stops after coverage "
        "exceeds 96% or repeated scans find no remaining frontier."
    ),
    Paragraph("4. Navigation and control", styles["Section"]),
    p(
        "The navigation layer combines direct goal seeking with Bug-style wall following. "
        "When the path is clear, differential wheel speeds steer the robot toward the "
        "target. When blocked, a PD controller maintains a compact wall-following distance "
        "while the controller searches for a safe route back toward the goal."
    ),
    p(
        "Loop detection tracks progress relative to the obstacle hit point. Returning "
        "close to that point after travelling away marks the current target unreachable "
        "and causes replanning. Bumper events, lack of displacement, and prolonged target "
        "pursuit also trigger reverse-and-turn recovery manoeuvres."
    ),
    Paragraph("5. Results and engineering observations", styles["Section"]),
    p(
        "The included complex scenario demonstrates autonomous traversal of corridors, "
        "diagonal barriers, and a spiral structure. The resulting occupancy map reached "
        "approximately 97.6% coverage in the recorded run. The most important practical "
        "improvements were:"
    ),
    ListFlowable(
        [
            ListItem(p(item))
            for item in (
                "Narrowing the effective ultrasonic beam during map updates.",
                "Protecting established walls from noisy free-space measurements.",
                "Choosing middle-distance frontiers instead of simply the nearest cell.",
                "Detecting navigation loops and stalled motion.",
                "Throttling grid updates and display painting independently.",
            )
        ],
        bulletType="bullet",
        leftIndent=7 * mm,
        bulletFontName="Helvetica",
        bulletFontSize=8,
    ),
    Paragraph("6. Conclusion", styles["Section"]),
    p(
        "The project demonstrates an end-to-end autonomous exploration pipeline: sensing, "
        "probabilistic mapping, target selection, local navigation, and recovery. Its "
        "map-independent controller can run in both included arenas and provides a compact "
        "foundation for experimenting with alternative frontier scoring, localisation "
        "sources, and path planners."
    ),
]

ReportDocTemplate(OUTPUT).build(story)
print(OUTPUT)
