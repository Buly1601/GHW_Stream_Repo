import pytest
from app import app as flask_app

@pytest.fixture
def app():
    """
    Make the env as testing.
    """
    flask_app.config.update({
        "TESTING": True,
    })

    yield flask_app

@pytest.fixture
def client(app):
    return app.test_client()