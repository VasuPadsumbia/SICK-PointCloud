#ifndef CUSTOMSTREAMBUFFER_H
#define CUSTOMSTREAMBUFFER_H

#include <streambuf>
#include <QTextEdit>

class CustomStreamBuffer : public std::streambuf
{
public:
    CustomStreamBuffer(QTextEdit* textEdit)
        : textEdit(textEdit)
    {
    }

protected:
    virtual int_type overflow(int_type v) override
    {
        if (v == '\n')
        {
            textEdit->append("");
        }
        else
        {
            textEdit->insertPlainText(QString(v));
        }
        return v;
    }

    virtual std::streamsize xsputn(const char* p, std::streamsize n) override
    {
        textEdit->insertPlainText(QString::fromUtf8(p, n));
        return n;
    }

private:
    QTextEdit* textEdit;
};

#endif // CUSTOMSTREAMBUFFER_H
