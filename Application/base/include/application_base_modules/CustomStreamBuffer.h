#include <streambuf>
#include <QPlainTextEdit>

class LiveConsoleStream : public std::streambuf
{
public:
    LiveConsoleStream(QPlainTextEdit* console) : console(console) {}

protected:
    int_type overflow(int_type v) override
    {
        if (v != EOF)
        {
            buffer += static_cast<char>(v);
            if (v == '\n')
            {
                updateConsole();
                buffer.clear();
            }
        }
        return v;
    }

    std::streamsize xsputn(const char* p, std::streamsize n) override
    {
        for (std::streamsize i = 0; i < n; ++i)
        {
            overflow(p[i]);
        }
        return n;
    }

private:
    void updateConsole()
    {
        QString text = QString::fromStdString(buffer).trimmed();

        // Replace last line
        QTextCursor cursor = console->textCursor();
        cursor.movePosition(QTextCursor::End);
        cursor.select(QTextCursor::LineUnderCursor);
        cursor.removeSelectedText();
        cursor.insertText(text);
        console->setTextCursor(cursor);
    }

    QPlainTextEdit* console;
    std::string buffer;
};
