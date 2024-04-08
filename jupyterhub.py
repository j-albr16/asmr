import os


REPO_URL = 'https://github.com/j-albr16/asumr'
BRANCH = 'main'
JUPYTERHUB_URL = 'https://jupyterhub.wwu.de'

# jupyterhub_url: https://jupyterhub.wwu.de/hub/user-redirect/git-pull?repo=https%3A%2F%2Fgithub.com%2Fj-albr16%2Fasumr&branch=main&urlpath=lab%2Ftree%2Fasumr%2Fasumr_exercises%2F


def url():

    url = input("""
Enter the path to the .ipynp file you want to open.

    Example: asumr_exercises/01_01_01.ipynb

url:
    """)

    url = f'lab/tree/asumr/{url}'

    if not url.endswith('/'):
        print('Take care! Added trailing / to url.')
        url = url + '/'

    if not url.startswith('asumr_exercises/'):
        print('Take care! You are not in the asumr_exercises folder.')

    # check if string contains .ipynb
    if not url.find('.ipynb') > 0:
        print('Take care! You did not enter a .ipynb file.')

    # encode : / -> %2F
    er_url = REPO_URL.replace('/', '%2F')
    url = url.replace('/', '%2F')

    # encode : : -> %3A
    er_url = er_url.replace(':', '%3A')
    url = url.replace(':', '%3A')

    # get redirect url
    redirect_url = f'{JUPYTERHUB_URL}/hub/user-redirect/git-pull?repo={er_url}&branch={BRANCH}&urlpath={url}'

    # print url
    print(redirect_url)


if __name__ == "__main__":
    url()
