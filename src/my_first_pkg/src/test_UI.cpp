#include <QApplication>
#include <QPushButton>
#include <QLabel>
#include <QGridLayout>
#include <QWidget>

class PlusMinus : public QWidget {
    Q_OBJECT

public:
    explicit PlusMinus(QWidget *parent = nullptr) : QWidget(parent) {
        // Erstelle die Buttons und das Label
        auto *plsBtn = new QPushButton("+", this);
        auto *minBtn = new QPushButton("-", this);
        lbl = new QLabel("0", this);

        // Erstelle ein GridLayout
        auto *grid = new QGridLayout(this);
        grid->addWidget(plsBtn, 0, 0);
        grid->addWidget(minBtn, 0, 1);
        grid->addWidget(lbl, 1, 1);

        setLayout(grid);

        // Verbinde Buttons mit den Funktionen
        connect(plsBtn, &QPushButton::clicked, this, &PlusMinus::OnPlus);
        connect(minBtn, &QPushButton::clicked, this, &PlusMinus::OnMinus);
    }

    ~PlusMinus() {}

private slots:
    void OnPlus() {
        int val = lbl->text().toInt();
        val++;
        lbl->setText(QString::number(val));
    }

    void OnMinus() {
        int val = lbl->text().toInt();
        val--;
        lbl->setText(QString::number(val));
    }

private:
    QLabel *lbl;
};

#include "test_UI.moc"

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    // Erstelle das PlusMinus-Fenster und zeige es
    PlusMinus window;
    window.show();

    // Starte die Qt-Eventschleife
    return app.exec();
}
