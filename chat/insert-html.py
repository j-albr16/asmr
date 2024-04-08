import os


HTML_PATH = "_build/html"
INCLUDE_HTML_PATH = "chat/chat_button.html"
MODAL_PATH = "chat/chat_modal.html"


def insert_html(html_path, include_html, modal_html):
    with open(html_path, "r") as f:
        html = f.read()

    html = html.replace(
        '<button onclick="toggleFullScreen()"',
        include_html + '\n<button onclick="toggleFullScreen()"'
    )

    html = html.replace(
        '</body>',
        modal_html + '\n</body>'
    )

    with open(html_path, "w") as f:
        f.write(html)


def main():

    with open(INCLUDE_HTML_PATH, "r") as f:
        include_html = f.read()

    with open(MODAL_PATH, "r") as f:
        modal_html = f.read()

    # traverse folder and subfolders
    for root, dirs, files in os.walk(HTML_PATH):
        for file in files:
            if file.endswith(".html"):
                html_path = os.path.join(root, file)
                insert_html(html_path, include_html, modal_html)

    # copy folder
    os.system("rm -rf _build/html/chat")
    os.system("cp -r chat _build/html/chat")


if __name__ == '__main__':
    main()
