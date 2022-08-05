
THEME = 'attila'
AUTHOR = 'DLR-TS'
SITENAME = 'Automated Driving Open Research (ADORe)'
SITEURL = ''


HEADER_COVER = 'images/ts-bs-fascar-start-image.jpg'
#HEADER_COLOR = 'black'
DISPLAY_PAGES_ON_MENU = True


PATH = 'content'

TIMEZONE = 'Europe/Berlin'

DEFAULT_LANG = 'en'

# Feed generation is usually not desired when developing
FEED_ALL_ATOM = None
CATEGORY_FEED_ATOM = None
TRANSLATION_FEED_ATOM = None
AUTHOR_FEED_ATOM = None
AUTHOR_FEED_RSS = None

# Blogroll
#LINKS = (('Pelican', 'https://getpelican.com/'),
#         ('Python.org', 'https://www.python.org/'),
#         ('Jinja2', 'https://palletsprojects.com/p/jinja/'),
#         ('You can modify those links in your config file', '#'),)

# Social widget
SOCIAL = (('twitter', 'https://https://www.dlr.de/ts/en'),
          ('github', 'https://github.com/eclipse/adore'),
          ('envelope','mailto:opensource-ts@dlr.de'),)


DEFAULT_PAGINATION = False

STATIC_PATHS = [
    'images',
    'extra',  # this
]
EXTRA_PATH_METADATA = {
    'extra/robots.txt': {'path': 'robots.txt'},
    'extra/favicon.ico': {'path': 'favicon.ico'},  # and this
    'extra/CNAME': {'path': 'CNAME'},
    'extra/LICENSE': {'path': 'LICENSE'},
    'extra/README': {'path': 'README'},
}



# Uncomment following line if you want document-relative URLs when developing
#RELATIVE_URLS = True
