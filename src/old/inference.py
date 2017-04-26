import numpy as np
import matplotlib.pyplot as plt
#from __future__ import division
#matplotlib inline
import scipy.stats as st
import seaborn as sns
sns.set(style='ticks', palette='Set2')

def bern_post(n_params=100, n_sample=100, true_p=.8, prior_p=.5, n_prior=100):
    params = np.linspace(0, 1, n_params)
    sample = np.random.binomial(n=1, p=true_p, size=n_sample)
    likelihood = np.array([np.product(st.bernoulli.pmf(sample, p)) for p in params])
    #likelihood = likelihood / np.sum(likelihood)
    prior_sample = np.random.binomial(n=1, p=prior_p, size=n_prior)
    prior = np.array([np.product(st.bernoulli.pmf(prior_sample, p)) for p in params])
    prior = prior / np.sum(prior)
    posterior = [prior[i] * likelihood[i] for i in range(prior.shape[0])]
    posterior = posterior / np.sum(posterior)
    
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(8,8))
    axes[0].plot(params, likelihood)
    axes[0].set_title("Sampling Distribution")
    axes[1].plot(params, prior)
    axes[1].set_title("Prior Distribution")
    axes[2].plot(params, posterior)
    axes[2].set_title("Posterior Distribution")
    sns.despine()
    plt.tight_layout()
    plt.show()
    return posterior

def bern_beta(n_flips, n_heads, prior_alpha, prior_beta):
    n_tails = n_flips - n_heads
    x_values = np.linspace(0, 1, 100)
    likelihood = [(x**n_heads) * ((1-x)**(n_tails)) for x in x_values]
    prior = [st.beta.pdf(x, prior_alpha, prior_beta) for x in x_values]
    posterior = [st.beta.pdf(x, prior_alpha + n_heads, prior_beta + n_tails) for x in x_values]
    
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(8,8))
    axes[0].plot(x_values, likelihood)
    axes[0].set_title("Sampling Distribution")
    axes[1].plot(x_values, prior)
    axes[1].set_title("Prior Distribution")
    axes[2].plot(x_values, posterior)
    axes[2].set_title("Posterior Distribution")
    sns.despine()
    plt.tight_layout()
    plt.show()

#example_post = bern_post()
#print example_post
bern_beta(n_flips=100, n_heads=10, prior_alpha=50, prior_beta=50)
